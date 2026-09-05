"""
 * @file tg_log.py
 * @brief Shared Python logging setup with an optional Telegram handler.
 *
 * Provides a standard-library `logging` configuration for all project host
 * tools (changelogger, CAN test host, ST-Link uploader, ...) plus a
 * TelegramHandler that mirrors log records into a Telegram chat through the
 * official Bot API.
 *
 * Credentials (bot token and chat id) are NEVER stored in the repository.
 * They are resolved in this order:
 *   1. Environment variables: TG_BOT_TOKEN and TG_CHAT_ID
 *   2. JSON file: <repo>/private/telegram_credentials.json
 *      {"bot_token": "...", "chat_id": "..."}
 * The `private/` directory is already listed in .gitignore, so the secrets
 * cannot be committed accidentally.
 *
 * Design goals:
 * - Zero third-party dependencies (urllib is used for HTTP).
 * - Never crash the host tool: any network/API error is reported to stderr
 *   once and the handler disables itself.
 * - If no credentials are found, Telegram logging is silently skipped and
 *   only console logging remains active.
 *
 * Usage:
 *   import tg_log
 *   log = tg_log.setup_logging("changelogger")   # Console + Telegram
 *   log.info("Build finished")                   # Goes to console and chat
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
"""

import json                                     # Credentials file parsing
import logging                                  # Standard logging framework
import os                                       # Paths and environment variables
import sys                                      # stderr for soft failures
import urllib.error                             # HTTP error handling
import urllib.parse                             # Form-encoding of the payload
import urllib.request                           # Bot API HTTPS requests

# Absolute path of the gitignored credentials file
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CREDENTIALS_PATH = os.path.join(_REPO_ROOT, "private", "telegram_credentials.json")

# Telegram Bot API endpoint template (token is substituted per request)
_API_URL = "https://api.telegram.org/bot{token}/sendMessage"
# Telegram message hard limit; longer records are split into chunks
_MAX_MESSAGE_LEN = 4000


def load_credentials():
    """Resolve bot token and chat id.

    Environment variables take priority over the JSON credentials file so CI
    machines can inject secrets without touching the working tree.

    @return (bot_token, chat_id) tuple, or (None, None) when unavailable
    """
    # 1. Environment variables (highest priority)
    token = os.environ.get("TG_BOT_TOKEN")
    chat_id = os.environ.get("TG_CHAT_ID")
    if token and chat_id:
        return token, chat_id

    # 2. Gitignored credentials file
    try:
        with open(_CREDENTIALS_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)                                 # Parse JSON
        token = data.get("bot_token") or token                  # Fill gaps from env
        chat_id = data.get("chat_id") or chat_id
    except (OSError, ValueError):
        pass                                                    # No file / bad JSON - skip
    if token and chat_id:
        return token, chat_id
    return None, None                                           # Telegram disabled


class TelegramHandler(logging.Handler):
    """logging.Handler that posts every formatted record to a Telegram chat.

    HTTP failures are swallowed after a single stderr warning so that a
    missing network connection never breaks the calling tool. Messages longer
    than the Telegram limit are split into chunks.
    """

    def __init__(self, token: str, chat_id: str, level=logging.NOTSET):
        super().__init__(level)
        self.token = token                                      # Bot API token
        self.chat_id = chat_id                                  # Destination chat
        self._broken = False                                    # Disabled after failure

    def emit(self, record):
        """Format the record and push it to the Bot API (never raises)."""
        if self._broken:
            return                                              # Already failed once
        try:
            text = self.format(record)                          # Render via formatter
            for chunk in _chunk_text(text):                     # Respect message limit
                self._send(chunk)
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self._broken = True                                 # Disable further attempts
            print(f"[tg_log] Telegram send failed, disabling: {exc}",
                  file=sys.stderr, flush=True)

    def _send(self, text: str):
        """Perform one sendMessage request."""
        payload = urllib.parse.urlencode({                      # Form-encoded POST body
            "chat_id": self.chat_id,                            # Target chat
            "text": text,                                       # Message body
            "disable_notification": "true",                     # Silent push (no sound)
        }).encode("utf-8")
        url = _API_URL.format(token=self.token)                 # Endpoint with token
        req = urllib.request.Request(url, data=payload)         # POST request
        with urllib.request.urlopen(req, timeout=10) as resp:   # Blocking call, 10 s limit
            resp.read()                                         # Drain response body


def _chunk_text(text: str):
    """Split text into Telegram-safe chunks by the message length limit."""
    for i in range(0, max(len(text), 1), _MAX_MESSAGE_LEN):     # Fixed-size windows
        yield text[i:i + _MAX_MESSAGE_LEN]


def setup_logging(name: str, level=logging.INFO,
                  telegram_level=logging.INFO, console=True) -> logging.Logger:
    """Configure and return a project logger with console + Telegram output.

    @param name            Logger name (usually the tool name)
    @param level           Console verbosity threshold
    @param telegram_level  Telegram verbosity threshold (records below it are
                           not mirrored to the chat)
    @param console         When False, suppress console output (Telegram only)
    @return Configured logger instance
    """
    logger = logging.getLogger(name)                            # Named logger per tool
    logger.setLevel(min(level, telegram_level))                 # Most verbose of the two
    logger.propagate = False                                    # Avoid duplicate root output
    if logger.handlers:                                         # Already configured
        return logger

    if console:                                                 # Console handler
        console_h = logging.StreamHandler()                     # stderr stream
        console_h.setLevel(level)                               # Console threshold
        console_h.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
        logger.addHandler(console_h)

    token, chat_id = load_credentials()                         # Resolve secrets
    if token and chat_id:                                       # Telegram configured
        tg_h = TelegramHandler(token, chat_id, level=telegram_level)
        tg_h.setFormatter(logging.Formatter("%(name)s:\n%(message)s"))
        logger.addHandler(tg_h)
    else:                                                       # No credentials available
        logger.debug("Telegram logging disabled: no credentials found")

    return logger
