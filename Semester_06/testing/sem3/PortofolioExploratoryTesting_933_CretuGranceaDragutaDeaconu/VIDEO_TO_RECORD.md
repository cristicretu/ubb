# Video to Record (3–5 minutes, your voice over)

**Chosen tool:** Bug Magnet — https://bugmagnet.org/
A Chrome / Firefox / Edge extension that right-click-injects exploratory test payloads (boundary strings, Unicode, dates, names with diacritics, XSS strings, big numbers, etc.) into any web form. Perfect for a short ET demo.

## Suggested scenario (~4 min)

**SUT idea (pick one):**
- A public sign-up / contact form on any web app you like — e.g. https://demoqa.com/automation-practice-form, https://practice.expandtesting.com/register, or a Vercel-deployed personal project.

**Charter:** *"Stress the input validation of the registration form to look for boundary, encoding, and length defects."*

## Outline for the recording

1. **00:00 – 00:30 — Intro.** Group 933, paper chosen (Afzal et al. 2015), why ET, why Bug Magnet (free, browser-native, no setup).
2. **00:30 – 01:00 — Install.** Show the Chrome Web Store page; install in 1 click; show the right-click menu now exposing **Bug Magnet ▸**.
3. **01:00 – 01:45 — Charter & approach.** Read the charter aloud, mention you will use heuristics from the **Names**, **Numbers**, **Text length** and **XSS** submenus, and keep a Rapid-Reporter-style spoken log of findings.
4. **01:45 – 03:30 — Execution (the bulk).** Live-test:
   - First name → Bug Magnet ▸ Names ▸ *"O'Brien"* — does the apostrophe break submission?
   - First name → *Names ▸ Long name* — is there a max length? Truncated silently? Error message?
   - Email → *Email addresses ▸ subdomain / plus-sign / IDN* — RFC-valid addresses that often fail naive regex.
   - Numeric field (age / phone) → *Numbers ▸ 2147483648* (int32 + 1), *negative*, *0*, *1.5*.
   - Free-text field → *XSS ▸ `<script>alert(1)</script>`* and *HTML injection* — does the page render or escape it?
   - Date field (if any) → *Dates ▸ 29 Feb non-leap*, *1900-01-01*.
   - For each: comment what you expected vs. what the form did, classify as bug / smell / pass.
5. **03:30 – 04:30 — Findings & wrap-up.** Recap defects found, severity (1/2/3), how Bug Magnet sped you up vs typing payloads manually, limits of the tool (web-only, no test management, no session export). One sentence linking back to the paper's finding that ET surfaces hard-to-reach defects efficiently.

## Recording tips

- **macOS:** built-in screen recording → `⌘ ⇧ 5`, choose *Record entire screen* or selected window, *Options ▸ Microphone ▸ MacBook Microphone*. Output is `.mov`, ~50 MB for 4 min — fine for Teams.
- **Browser:** use a fresh Chrome profile so the right-click menu is uncluttered.
- **Audio:** test mic levels for 5 seconds before the real take.
- **Filename:** save as `Demo_BugMagnet_933.mp4` (or `.mov`) and drop it into this folder; then re-run the zip step below.

## After recording

```fish
cd /Users/giga/fun/ubb/Semester_06/testing/sem3
rm -f PortofolioExploratoryTesting_933_CretuCretuDragutaDeaconu.zip
zip -r PortofolioExploratoryTesting_933_CretuCretuDragutaDeaconu.zip \
  PortofolioExploratoryTesting_933_CretuCretuDragutaDeaconu \
  -x "*VIDEO_TO_RECORD.md" "*.md"
```
