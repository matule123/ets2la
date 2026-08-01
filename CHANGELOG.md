# História verzií UltraPilot

Tento súbor je určený pre vývojový repozitár. Inštalátor ho nezahŕňa do
runtime balíka aplikácie.

## v0.4.2

- Stabilná identita hernej GPS navigácie oddelená od posúvajúceho sa SDK okna.
- Jednotná autorita revidovaného `lane_trajectory` snapshotu pre autopilota,
  HUD, AR, live mapu a `nav_path`.
- Opravené dočasné výpadky LaneLocatora, prechody road/prefab, zmeny pruhov,
  merge/split a ochrana proti starým callbackom.
- Stabilnejšie vedenie po strednici pruhu, plynulejšie riadenie a prispôsobenie
  rýchlosti zákrutám.
- Rozšírená diagnostika výpočtu trasy so stabilnými failure kódmi a bezpečným
  anonymizovaným exportom.
- Prepracovaný HUD, AR, navigačná stránka, live mapa, nastavenia, pluginy,
  aktualizačné okno a výkonový panel.
- Nový bezpečnejší aktualizačný a inštalačný tok vrátane filtrovaného runtime
  balíka, presného priebehu sťahovania a opravy SDK.
- Opravená čistá inštalácia Python závislostí: zlyhanie `vgamepad` sa už
  neoznačí ako úspech a Engine nevstúpi do nekonečnej reštartovacej slučky.
- Onboarding zobrazuje vlajky jazykov a presnú chybu načítania mapových balíkov.
- Nainštalovaná aplikácia uchováva a zobrazuje presné číslo revízie.

## v0.4.1

- Prvá verejná vývojová verzia UltraPilotu.
- Základ autopilota, telemetrie ETS2, HUD a AR zobrazenia.
- Herná GPS navigácia, mapové datasety a prvá verzia výpočtu trasy.
- Správa pluginov, nastavenia aplikácie, onboarding a aktualizačný systém.
- Základný inštalátor so SDK pluginmi a virtuálnym ovládačom.

