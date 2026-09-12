# =============================================================================
# @file protect_main.ps1
# @brief Enables branch protection on 'main' requiring a green CI build
#
# Calls the GitHub REST API to mark the "Build Firmware" status checks
# (.github/workflows/build.yml) as required: GitHub will refuse to merge a
# pull request into main until both firmware environments compile.
#
# Usage:
#   .\tools\protect_main.ps1 -Token <github_pat>
#
# The token needs the "Administration" (repository) permission and may belong
# to a fine-grained PAT or be a classic PAT with the "repo" scope. The token
# is used only for this single request and is never stored.
#
# Equivalent manual setup: GitHub -> Settings -> Branches -> Add branch
# protection rule -> Branch name pattern "main" -> enable "Require a pull
# request before merging" + "Require status checks to pass" -> select
# "Build Firmware / build (cln17_v3_release)" and
# "Build Firmware / build (cln234_release)".
#
# Copyright 2026, creapunk
# Licensed under the Apache License, Version 2.0
# =============================================================================

param(
    [Parameter(Mandatory = $true)]  # Fine-grained PAT with Administration permission, or classic PAT with 'repo' scope
    [string]$Token,

    [string]$Owner = "dVNaNt",      # Repository owner (github.com/<owner>/<repo>)
    [string]$Repo = "TP_FW",        # Repository name
    [string]$Branch = "main",       # Protected branch
    [string[]]$RequiredChecks = @(  # CI checks that must pass before merge
        "Build Firmware / build (cln17_v3_release)",
        "Build Firmware / build (cln234_release)"
    )
)

# Build the branch protection payload: PR-only workflow + required status checks
$body = @{
    required_status_checks = @{
        strict   = $false               # Do not require the branch to be up to date with main
        contexts = $RequiredChecks      # Names of the CI checks that gate the merge
    }
    enforce_admins                 = $false   # Allow admins to bypass (set $true to enforce for everyone)
    required_pull_request_reviews  = @{
        required_approving_review_count = 1   # At least one approval besides the green build
    }
    restrictions                   = $null    # No push restrictions
    allow_force_pushes             = $false   # Forbid history rewriting on the protected branch
    allow_deletions                = $false   # Forbid deleting the protected branch
} | ConvertTo-Json -Depth 5

# PUT the protection rule to the GitHub API
$uri = "https://api.github.com/repos/$Owner/$Repo/branches/$Branch/protection"
$headers = @{
    Authorization  = "Bearer $Token"              # Authenticate the request
    Accept         = "application/vnd.github+json"
    "X-GitHub-Api-Version" = "2022-11-28"         # Pin the API version for stability
}

try {
    $response = Invoke-RestMethod -Method Put -Uri $uri -Headers $headers -Body $body -ContentType "application/json"
    Write-Host "OK: '$Branch' is protected. Required checks:" -ForegroundColor Green
    $response.required_status_checks.contexts | ForEach-Object { Write-Host "  - $_" }
} catch {
    Write-Error "Failed to update branch protection: $($_.Exception.Message)"
    if ($_.ErrorDetails.Message) { Write-Error $_.ErrorDetails.Message }  # Surface the API error body
    exit 1
}
