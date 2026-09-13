import argparse
import datetime
import os
import re
import sys                                      # Path manipulation for imports

# Project root is one level above this script (changelog/)
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "tools"))  # Make tools/ importable by path
import tg_log                                   # Console + Telegram logging setup

MAX_FLASH = 114688   # board_upload.maximum_size: 128 KiB minus the 16 KiB NVM reservation, same base as PlatformIO
MAX_RAM = 32768

def get_previous_sizes(filepath):
    if not os.path.exists(filepath):
        return None, None
        
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
        
    flash_matches = re.findall(r'FLASH:\s*(\d+)\s*bytes', content)
    ram_matches = re.findall(r'RAM:\s*(\d+)\s*bytes', content)
    
    prev_flash = int(flash_matches[-1]) if flash_matches else None
    prev_ram = int(ram_matches[-1]) if ram_matches else None
    
    return prev_flash, prev_ram

def format_as_list(text):
    # Split the text by ';' or newlines to build a markdown list
    items = [item.strip() for item in re.split(r'[;\n]', text) if item.strip()]
    if not items:
        return "- None"
    return "\n".join(f"- {item}" for item in items)

def format_delta(delta):
    return f"+{delta}" if delta >= 0 else str(delta)

def main():
    parser = argparse.ArgumentParser(description="Markdown Changelog Generator")
    parser.add_argument("summary", help="Short summary of the update")
    parser.add_argument("changes", help="List of changes (separate with ';')")
    parser.add_argument("reason", help="List of reasons (separate with ';')")
    parser.add_argument("notes", help="List of notes (separate with ';')")
    parser.add_argument("flash", type=int, help="Current FLASH size in bytes")
    parser.add_argument("ram", type=int, help="Current RAM size in bytes")
    
    args = parser.parse_args()
    
    # Changelog file lives next to the script (changelog/changelog.md)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    changelog_path = os.path.join(script_dir, "changelog.md")
    
    prev_flash, prev_ram = get_previous_sizes(changelog_path)
    
    delta_flash = args.flash - prev_flash if prev_flash is not None else 0
    delta_ram = args.ram - prev_ram if prev_ram is not None else 0
    
    flash_pct = (args.flash / MAX_FLASH) * 100
    ram_pct = (args.ram / MAX_RAM) * 100
    
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    
    entry = f"""## {date_str} - {args.summary}
**Changes:**
{format_as_list(args.changes)}

**Reasoning:**
{format_as_list(args.reason)}

**Notes:**
{format_as_list(args.notes)}

FLASH: {args.flash} bytes ({flash_pct:.2f}% of {MAX_FLASH} bytes): {format_delta(delta_flash)} bytes
RAM: {args.ram} bytes ({ram_pct:.2f}% of {MAX_RAM} bytes): {format_delta(delta_ram)} bytes
"""

    with open(changelog_path, 'a', encoding='utf-8') as f:
        f.write(entry + "\n\n")
        
    # Mirror the report to console and the Telegram chat via the shared logger
    log = tg_log.setup_logging("changelogger")
    log.info("📦 FLASH: %d bytes (%s bytes)\n📦 RAM:   %d bytes (%s bytes)",
             args.flash, format_delta(delta_flash),
             args.ram, format_delta(delta_ram))
    log.info(entry)                             # Full changelog entry to Telegram

if __name__ == "__main__":
    main()