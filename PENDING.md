# Pending items

Status as of 2026-11-01. Board: Waveshare ESP32-S3-Touch-AMOLED-1.8.

---

## atomberg_remote — Atomberg fan remote

- [ ] **Verify after the daily API limit resets**: fans load, on/off works reliably, and the fan list
      survives a reboot (saved in NVS).
- [ ] Find the actual daily request limit on the Atomberg developer portal; tune `POLL_MS` if needed.
- [ ] Optional: local control without the cloud — listen on UDP 5625 to see what the fans broadcast;
      IR LED fallback (NEC, address `0xF300`) if local Wi-Fi control isn't possible.
- [ ] Consider rotating the API key and refresh token (they were shared in chat).

---

## flasher — local flashing web page

- [ ] First real use: flash a project from the page (only the server endpoints were smoke-tested).
