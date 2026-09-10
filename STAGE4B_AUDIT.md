# UltraPilot — Etapa 4B: kalibrácia riadiaceho aktuátora a odozva pri rýchlosti

Dátum: 2026-09-10. Zdrojový základ: `e53ee18` (Etapa 4A). V tejto práci nebol
vykonaný commit, push, inštalácia, kopírovanie do prevádzkovej aplikácie ani
reštart hry/aplikácie.

## Verdikt

**ETAPA 4B JE OFFLINE DOKONČENÁ — READY FOR ETAPU 4C.** Pre aktuálne
zaznamenaný truck/input setup je kalibrácia statická, explicitná, validovaná a
naviazaná na každý riadiaci packet. Namerané časovanie je oddelené od návrhového
tlmenia regulátora. Nepribudol filter, online adaptácia, druhý regulátor ani
ďalší fyzický limiter.

**Celý autopilot ešte nie je schválený na bežnú prevádzku.** Phase 4C má
dokončiť regulátor a až potom má nasledovať kontrolovaný test v ETS2. Mimoriadne
pomalý, ale aktuálnymi replaymi nepotvrdený plant 0,60 s zostáva otvorený stress
scenár. R18 s návesom nie je touto etapou vyhlásený za bezpečný; swept-envelope
planner patrí do ďalšej samostatnej etapy.

Etapa nemení LanePath, GPS UID, LaneId, mapové dáta, HUD, AR, live mapu,
brzdenie, trailer referenciu, SteeringDynamics ani bezpečnostné limity.

## 1. Dôkazové vstupy

Použité boli dva najnovšie oddelené frame-bound exporty tej istej mapy,
session a datasetu. Intent a build sa zámerne nemiešajú; každý export sa najprv
analyzuje samostatne a až fyzikálne hodnoty aktuátora sa agregujú.

| Jazda | Intent / build | Revision | Aktívne jedinečné SDK frames | SHA-256 |
|---|---|---:|---:|---|
| Bez návesu | `706cf703…` / `8935a0dc…` | 8 | 2 259 | `f0fe9ed1a121c2233b4c15805d45e01027e725960cc46ea2fd5df019f6c53677` |
| S návesom | `be5f3de0…` / `5c958dd1…` | 8 | 2 136 | `955b3591fd0cba091772be53a1431004427285c4396eefae38e4f6ceeb8ce471` |

Spoločné údaje: session 1, mapa `promods-1.59`, fingerprint
`d6cc7936fce4e902761abb5d`. Spolu sa vyhodnotilo 4 395 jedinečných aktívnych
SDK snímok. Duplicitné aplikačné tick-y nad rovnakým SDK frame sa nepovažujú za
nové fyzikálne meranie. Chýbajúce frames sa neinterpolujú.

Reprodukovateľný výsledok je v ignorovanom pracovnom výstupe
`docs/steering-audit/stage4b-actuator-identification.json`; zdrojový read-only
nástroj je `tools/audit_steering_actuator.py`.

## 2. Nájdené koreňové príčiny

### 2.1 Nesprávny predpoklad o pomalej hernej odozve

Pred etapou bola fyzická predikcia zostavená ako približne 0,32 s herná odozva
+ 0,10 s transport + 0,03 s pozorovacia fáza, spolu 0,45 s. Dense replay to
nepotvrdzuje. `gameSteer` najlepšie zodpovedá predchádzajúcemu aplikovanému
engine commandu — teda jednému SDK frame, bez dokázaného ďalšieho 0,32 s lag.

| Zarovnanie command → gameSteer | RMS všetky [input] | RMS dynamické [input] |
|---|---:|---:|
| Rovnaký SDK frame | 0,005557 | 0,009481 |
| **O jeden frame** | **0,001837** | **0,003265** |
| O dva frames | 0,004494 | 0,005720 |
| O tri frames | 0,007458 | 0,009601 |

Medián nového SDK frame je 0,066664 s. Calculation → application frame lag má
medián tiež 0,066664 s. Preto je fyzický horizont pozorovanie → účinok
`0,067 + 0,067 = 0,134 s`, nie 0,45 s. Packet age má medián 0,043736 s;
nepoužíva sa namiesto SDK frame identity.

### 2.2 Jedna premenná riadila dve rozdielne veci

Pôvodný `actuator_response_s` súčasne:

1. posúval curvature preview o `v * response`,
2. menil priestorovú dĺžku a teda gain heading/CTE feedbacku.

Pri pokuse opraviť časovanie by sa preto neúmyselne zmenil aj regulátor a na
90 km/h by sa mohla zhoršiť rovina. Oprava tieto významy oddelila:

- `curvature_preview_s = 0,134 s` je meraná fyzická latencia;
- `feedback_response_s = 0,45 s` ostáva nezmenený návrhový člen tlmenia.

Phase 4B teda nemení CTE/heading pole placement. Definitívna práca na gainoch
patrí do 4C.

### 2.3 Starý odhad 0,78 rad/input neplatí pre túto jazdu

Pri 1 961 frame-bound vzorkách s `|gameSteer| >= 0,05` je priamy SDK prevod:

| `roadWheelAngle / gameSteer` | Hodnota [rad/input] |
|---|---:|
| Minimum | 0,698181 |
| P10 | 0,698214 |
| Medián | **0,698693** |
| P90 | 0,699762 |
| Maximum | 0,701338 |
| Ľavé zákruty — medián | 0,698445 |
| Pravé zákruty — medián | 0,698965 |

Preto ostáva konzervatívny runtime default 0,70 rad/input. Nie je to
univerzálna ETS2 konštanta. Structured settings umožňujú explicitnú hodnotu
0,60–0,95; neplatná hodnota je fail-closed.

| Rýchlosť | Počet priamych vzoriek | Medián [rad/input] |
|---|---:|---:|
| 0–20 km/h | 263 | 0,699194 |
| 20–30 km/h | 1 167 | 0,698902 |
| 30–40 km/h | 420 | 0,698279 |
| 40–50 km/h | 111 | 0,698197 |
| 50–65 km/h | 0 s dostatočným natočením | nepreukázané |
| 65–95 km/h | 0 | nepreukázané |

Yaw diagnostika poskytuje nepriamu kontrolu vyššie: medián
`(yaw/v) / (tan(tyre)/3,8)` je 1,0111; P10–P90 0,9543–1,0583. V pásme
50–65 km/h je medián 1,0102 zo 157 vzoriek. Yaw však zahŕňa sklz, preto nikdy
nemení kalibráciu online.

### 2.4 Diagnostika porovnávala časovo rozdielne príkazy

Sekundový log počítal `game_tracking` a command tyre angle z práve vypočítaného
plugin outputu, hoci `gameSteer` patrí poslednému commandu flushnutému Engine.
Diagnostika teraz používa `engine_applied_steering`; dense replay navyše ukladá
osobitne calculation a application kalibráciu. Táto zmena nemení riadenie, iba
bráni ďalšiemu chybnému odhadu.

## 3. Nový dátový tok, rovnice a jednotky

```text
settings.steering_actuator_calibration
  -> validácia schémy, rozsahov a source
  -> Map: lock_rad + curvature_preview_s
  -> Route: rovnaký LanePath, kratší speed-scaled curvature preview
  -> packet obsahuje rovnakú calibration identitu
  -> Autopilot odmietne packet po zmene kalibrácie
  -> existujúci jediný SteeringDynamics
  -> Engine -> SCSControls -> game
```

Znamienka ostávajú nezmenené: tyre angle a normalized command sú kladné
doprava. Nové rovnice:

```text
delta_tyre [rad] = command [-] * K_delta [rad/input]
K_delta(default) = 0.70 rad/input

T_command = 0.067 s
T_observation = 0.067 s
T_preview = T_command + T_observation = 0.134 s
s_preview = s_current + |v [m/s]| * T_preview

feedback_length = max(8 m, wheelbase + |v| * 0.45 s)  # nezmenené
```

Pri 90 km/h sa curvature preview skráti z 11,25 m na 3,35 m. Pri 30 km/h z
3,75 m na 1,12 m. Nejde o filter ani držanie poslednej hodnoty: Route stále
každý tick vypočíta nový stateless curvature/CTE/heading command. Zmenila sa iba
fyzikálne nesprávna poloha budúcej curvature vzorky.

Structured konfigurácia má schému 1, source, tyre gain, command delay a
observation delay. Starý scalar `steering_lock_rad` je podporovaný iba ako
migračný vstup, keď structured objekt úplne chýba. Ak objekt existuje, scalar
ho nikdy neprepíše. Chýbajúci source, neznáma schéma, NaN alebo rozsah mimo
validácie zablokujú riadenie s presným dôvodom. Online adaptácia je vždy false.

## 4. Uzavretý benchmark pred/po

Nezávislý nelineárny bicycle plant používa vlastný wheelbase/gain/lag/transport,
5 ms vnútorný integračný krok, 20 Hz controller, jitter, oneskorený tick a šum.
Pozorovaný bod je 2,1 m pred zadnou nápravou podľa Etapy 4A. Rovnaký plant a
rovnaký regulátor sa porovnáva iba s preview 0,45 vs 0,134 s.

### Plant zodpovedajúci poslednému dense meraniu

Tabuľka uvádza horší ľavý/pravý variant. R18 je iba test laterálneho sledovania
kabíny; nie je to swept-envelope potvrdenie bezpečnosti návesu.

| Scenár | Max CTE 0,45 → 0,134 s [m] | RMS CTE 0,45 → 0,134 s [m] | Monotónne opačné vzorky po |
|---|---:|---:|---:|
| R250 / 90 km/h | 0,258 → **0,075** | 0,085 → **0,025** | 0 |
| R83 | 0,236 → **0,094** | 0,087 → **0,033** | 0 |
| R35 | 0,332 → **0,155** | 0,117 → **0,052** | 0 |
| R18 | 0,571 → **0,330** | 0,195 → **0,113** | 0 |
| S35 | 0,641 → **0,292** | 0,180 → **0,079** | 0 |
| Kruhový R25 | 0,425 → **0,227** | 0,133 → **0,071** | 0 |

Všetky scenáre dokončili trasu, nevznikla strata LaneMatch simulovaná vlastným
riadením a žiadny monotónny oblúk nedostal opačný command. Cab/trailer variant
má v tejto etape rovnaké riadenie: trailer offset je po predchádzajúcej etape
zámerne nulový a planner ešte nie je implementovaný.

### Roviny bez regresie

| Rýchlosť | Max CTE pred = po [m] | RMS CTE pred = po [m] |
|---|---:|---:|
| 10 km/h | 0,500 | 0,0973 |
| 30 km/h | 0,500 | 0,0983 |
| 60 km/h | 0,500 | 0,1174 |
| 90 km/h | 0,500 | 0,1358 |

Max 0,5 m je vložená počiatočná odchýlka. Výsledky pred/po sú zhodné, čo
dokazuje, že skrátenie curvature preview nezmenilo feedback gain na rovine.

### Nezávislé stress modely

| Plant | Výsledok po | Interpretácia |
|---|---|---|
| lag 0,25 s + transport 0,10 s | všetky dokončili, 0 loss, 0 monotónnych opačných vzoriek; najhoršia krivka max CTE 0,188 m | rezerva nad nameranú odozvu |
| lag 0,60 s + transport 0,10 s pri default preview 0,134 s | všetky dokončili, 0 loss; max CTE 0,656 m; R250 má 9 krátkych opačných recovery vzoriek | **otvorené riziko; taký plant replay nepotvrdzuje** |
| gain plantu 0,60 / config 0,70 | 0 loss; najhoršie R18 0,945 m | nesprávna konfigurácia viditeľne zhoršuje presnosť |
| gain plantu 0,95 / config 0,70 | 0 loss; najhoršie R18 1,075 m | dôvod kalibráciu nevydávať za univerzálnu |

Veľmi pomalý plant sa zámerne neskryl. Pri preukázanom inom setup-e treba
zadať inú statickú latenciu a znovu spustiť benchmark; runtime sa nesmie
prispôsobiť podľa rastúcej CTE. Phase 4C musí navyše overiť regulátor na tejto
hranici. Bez nových reálnych dát sa default nemení smerom k extrémnemu stressu.

Kompletných 210 behov je v ignorovanom
`docs/steering-audit/stage4b-benchmark.json`.

## 5. Bezpečnosť a stavová autorita

- Kalibrácia je súčasťou calculation packetu.
- Autopilot porovnáva schema, gain, obe latencie a source s aktuálnym stavom.
- Packet zo starej kalibrácie sa odmietne; nepoužije sa na novú revision.
- Existujúce intent/revision/build/session/map/dataset a monotónne časové
  kontroly zostali nezmenené.
- `SteeringDynamics` ostáva jediným command/rate/acceleration stupňom.
- Road-wheel angle a yaw sú iba diagnostika. Nevznikol observer ani adaptívny
  feedback do volantu.
- Invalid structured kalibrácia sa nevracia na legacy scalar ani default.
- Recorded route nepreberá aktívnu GPS autoritu.

## 6. Zmenené súbory

| Súbor | Účel |
|---|---|
| `core/steering_calibration.py` | Nová immutable schéma, validácia, legacy migrácia a fail-closed dôvody. |
| `core/settings/manager.py` | Čistá inštalácia dostane structured kalibráciu 0,70 / 0,067 / 0,067. |
| `core/lateral_controller.py` | Oddeľuje návrhových 0,45 s feedbacku od fyzickej latencie. |
| `core/navigation/route.py` | Samostatný curvature preview; diagnostika oboch časov; presný calibration failure. |
| `plugins/map/main.py` | Načíta a publikuje jednu kalibráciu, odovzdá gain/preview Route a bindne ju do packetu. |
| `plugins/autopilot/main.py` | Odmietne stale calibration packet; rozšíri dense diagnostiku; opraví časové párovanie logu. |
| `tests/steering_bench.py` | Voliteľný explicitný preview pre rovnaký nezávislý plant; staré defaulty ostali. |
| `tests/test_stage4b_actuator_calibration.py` | 12 nových testov / 22 podtestov pre schému, stale packet, reálne hodnoty a closed-loop. |
| `tools/audit_steering_actuator.py` | Read-only frame alignment a identifikácia gain/delay/yaw. |
| `tools/run_stage4b_bench.py` | Reprodukovateľných 210 before/after/stress behov. |
| `STAGE4B_AUDIT.md` | Tento report. |

Žiadny starý test nemal zmenenú toleranciu, očakávané znamienko, fyzikálny
model ani očakávaný výstup. Jediná zmena starého testovacieho súboru je nový
voliteľný argument `controller_preview_s`; bez neho sa každý existujúci test
správa byte-for-byte rovnako.

## 7. Overenie

- Nové testy Etapy 4B: **12 prešlo, 22 podtestov prešlo**.
- Cielená steering/map/replay integrácia: **107 testov, 1 050 podtestov**.
- Celá sada po finálnej implementácii: **607 testov, 1 315 podtestov**.
- `compileall core plugins tests tools`: prešlo.
- `git diff --check`: prešlo; iba existujúce Windows LF/CRLF upozornenia.
- Benchmark: 210 nezávislých behov.

Prvý full pytest v sandboxe dobehol testovú časť, ale Windows odmietol čítanie
pytest basetemp pri session cleanup. Platný finálny výsledok je opakovaný beh
mimo sandboxu s novým basetemp; nejde o zamaskované test failure.

## 8. Ďalší krok a plán neskoršieho ETS2 testu

Ďalší vývojový krok je **Etapa 4C — finálne doladenie jediného laterálneho
regulátora** nad novou geometriou 4A a kalibráciou 4B. Etapa 4C nesmie znovu
spojiť fyzickú latenciu s feedback gainom ani pridať smoothing.

Po 4C a používateľom vykonanej inštalácii:

1. Overiť `actuator_calibration.valid=true`, source a `preview_horizon_s=0.134`.
2. Overiť `reference_geometry.valid=true` z 4A.
3. Bez návesu: rovina 20–30 km/h, dlhá ľavá/pravá, S-zákruta, výjazd.
4. Potom 40–60 km/h. 90 km/h iba na bezpečnom úseku; dnešné priame tyre gain
   vzorky nad 50 km/h nemajú dostatočné natočenie na samostatnú identifikáciu.
5. Manuálne vypnúť autopilot a zachovať celý nový dense replay.
6. Samostatne test s návesom; R18/90° sa nesmie označiť za bezpečný bez
   swept-envelope planneru.

V logu sledovať calculation/application calibration, `curvature_preview_s`,
`feedback_response_s`, calculation/application SDK frame, packet age/frame
lag, `engine_applied_steering`, `game_steer_right`, `tyre_angles_rad`, yaw,
`lane_cte_m`, body tracking error, local/preview curvature, steer raw/out,
LaneId a úplnú trajectory identitu.
