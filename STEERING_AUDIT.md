# Audit riadenia UltraPilot — 2026-09-05

## Stav a rozsah dôkazu

Zdrojový základ bol čistý commit `e8d5c57`. Pred zásahom prešlo 482 testov.
Pracovalo sa iba v tomto repozitári; referencie, inštalácia, log a video boli
čítané. Nebol vykonaný commit, push, inštalácia ani reštart.

Riadenie je matematicky prepracované a offline overené. **Nie je to potvrdenie
bezchybnej jazdy v ETS2 ani splnenia všetkých návesových podmienok.** Na výjazde
R18 v 4,5 m pruhu zostáva približne 8 cm prekročenie modelovej bočnej obálky
návesu. Nižšie je tento výsledok uvedený rovnako ako zlepšenia.

## Referencia ETS2LA a porovnanie toku

Preskúmané read-only adresáre:

- `C:\Users\PC\Downloads\ets2la aplikácia\ETS2LA\app`
- `C:\Users\PC\Downloads\Euro-Truck-Simulator-2-Lane-Assist-main (1)\Euro-Truck-Simulator-2-Lane-Assist-main`

Obe referencie obsahujú ETS2LA 0.5.2; relevantné implementácie sú zhodné.
Map plugin má vlastnú verziu 2.0.0 a limit 20 Hz. Licencia je GPLv3; kód ani
assety sa nekopírovali. `Plugins/Map/route/driving.py` má SHA256
`0d9c277a35937c46840f9b663fcc9672e8851aa837f5a77467d0152a233fbcf8`.
Samotný názov algoritmu ani referencia nedokazujú spoľahlivosť vo všetkých hrách.

| Krok / autorita | ETS2LA referencia | UltraPilot pred auditom | UltraPilot po audite |
|---|---|---|---|
| Pozorovanie | TruckSimAPI, jedna sada SDK dát; X/Z metre, Y výška, rotácia v otáčkach → radiány | Engine 60 Hz podľa nastavenia; Map čítal polohu, heading a rýchlosť oddelene | Jeden existujúci `vehicle_envelope_snapshot` pre XYZ, heading, rýchlosť a yaw; SDK frame ID |
| Trasa | Map planning, directed road/prefab body, postupné odstránenie bodov za vozidlom | Revisionovaný LanePath, LaneMatch, GPS intent | Nezmenené; žiadna nová geometrická autorita |
| Progres | Najbližší dopredný bod; orezanie za vozidlom, obmedzenie vzdialenosti | Directed projekcia, cache progresu a occurrence, presný LaneId/deck | Nezmenené pravidlá projekcie a hraníc |
| Lokálna referencia | Prvých päť dopredných bodov, krátka sečnica | Päť pursuit horizontov okolo `4 + 0,912*v`, maximum 28 m; low-speed capture | Krátka lokálna tangenta; curvature preview iba o `v*0,42 s`, nepresúva bod merania CTE |
| Bočná chyba | Podpísaný cross product voči sečnici, metre | LaneMatch CTE otočené raz do Route konvencie; pursuit geometria | Rovnaké znamienko, jeden Frenet error; žiadny druhý Stanley/PID |
| Heading | Podpísaný uhol sečnice, stupne | Bearing pursuit targetu, diagnóza lokálnej tangenty | Radiány voči lokálnej tangente tej istej potvrdenej geometrie |
| Riadiaci zákon | `(heading_deg + 7,5*lateral_m)*M/180`, M klesá z 8 na 2; empirická škála | Inverzný bicycle prevod s predpokladom 0,28 rad/normalizovaný vstup | Jedna požadovaná curvature: FF + heading + CTE; jediný inverse bicycle prevod |
| Náves | Pri malej rýchlosti interpolovaná poloha medzi trailerom a truckom, pri vyššej truck | Outward offset zo spatial curvature, ale bez zodpovedajúcej tangenty offsetovej referencie | Poloha, tangenta aj zakrivenie jednej virtuálnej tractor referencie; trailer CTE je len diagnóza |
| Výstup | Map clamp ±0,95, Steering sensitivity 1,2, vážená história 0,2 s, SDK | Route clamps + SteeringDynamics; game actuator navyše | Route iba API rozsah ±1; jediný SteeringDynamics, bez jeho deadbandu |
| Čas / stav | Map cap 20 Hz; perf_counter; Steering história vzoriek | Plugin cieľ 100 Hz (10 ms spánok + výpočet), wall clock dt; Engine samostatne | Frekvencie nezmenené, Engine aj plugin používajú monotónne dt |
| Aktivácia | Map enabled/takeover, Steering sendToGame | Dynamics sync z `-gameSteer` iba v bežnej vetve | Rovnaký sync aj pred early-return brzdením |
| Brzdenie | ACC/SDK longitudinal osobitne | Emergency čítal scalar samostatne; PAY_TOLL nechával starý volant | Cruise/emergency/toll používajú ten istý overený packet a jeden actuator |
| SDK výstup | ETS2LAPluginInput preferovaný, s negáciou; fallback SCSControls iné ABI | Engine jediný writer SCSControls/vgamepad | Nezmenené; nemožno slepo prevziať ETS2LA negácie medzi odlišnými ABI |

ETS2LA forward je rovnako `(-sin(h), -cos(h))`. Map vyberá do 50 bodov s
rastúcim rozostupom 0,25/2/4/8/16 m, odmieta skoky nad 20 m a nespoľahlivú
polohu. Tieto tolerancie sa do UltraPilotu neprenášali. V referencii nie je
explicitný fyzikálny curvature feed-forward; krátka sečnica ho empiricky
nahrádza. Jej stabilitu nemožno pripísať iba absencii filtra — filter tam je.

Pred auditom:

`SDK → Engine scalars → LaneMatch + LanePath → pursuit + trailer CTE offset → Route clamps → nav scalar → SteeringDynamics → Engine gate → SCSControls → game`

Po audite:

`SDK → jedna observation → rovnaký LaneMatch/LanePath → lokálny Frenet + differential trailer reference → jedna curvature demand → inverse calibration → jeden command packet → rovnaká authority gate → SteeringDynamics → Engine gate → SCSControls → game`

## Reálne dôkazy a obmedzenia synchronizácie

Log: `C:\Users\PC\AppData\Local\Programs\UltraPilot\ultrapilot.log`, SHA256
`40da83e6209c13524c1f6db39a6ecba4c91d0acf6df4827a342d1a18001b10bd`.
Posledná jazda je 16. 8. do 11:51:04, nie deň dostupného videa.

Z uvedených starších videí bolo teraz dostupné iba
`C:\Users\PC\Videos\Captures\Euro Truck Simulator 2 2026-08-15 22-23-37.mp4`.
Dekódovalo sa všetkých **7 569 snímok / 7 568 susedných párov**, posledné PTS
142,014983 s. Pre každý pár je uložený optical flow, rezíduum a čas, nie iba
vzorky každých päť sekúnd. Vizuálne sa preverili celkové indexy a husté
susedné snímky významných udalostí. Nie je poctivé zameniť toto za ručné
zmeranie fyzického uhla volantu v každej snímke. Pohľad zhora a pohyby kamery
sú neplatné pre meranie uhla. Video má variabilné FPS; export používa počítanie
dekódovaných snímok, nie nepresný seek podľa nominálneho FPS.

Priradenie `22:23:37 + PTS` je podľa názvu záznamu, s neistotou začiatku
záznamu a ~1 Hz diagnostiky. Nie je to frame-presná synchronizácia SDK.
Chýba spoločný timestamp v obraze a tick logu; sub-frame oneskorenie z nich
nemožno presne identifikovať.

| Dôkaz | Pozorovanie |
|---|---|
| 15. 8., PTS 45–53 s; log 916028–916039 | Ľavá zákruta rastie; CTE −0,276 → −0,818 m. FF −0,039 → −0,138, ale raw iba −0,014 → −0,053. Spätná korekcia systematicky ruší väčšinu základu. |
| PTS 55,227–56,252; 916041–916042 | FF ostáva −0,104, raw −0,060 → −0,042, game −0,060 → −0,041; CTE ~−0,63 m. Amplitúda sa mení už pred aktuátorom. |
| PTS 94,892–105,073; 916095–916111 | FF −0,341 až −0,606, zmeny LaneId na spojitej ceste, CTE až −0,844; výjazd a odvíjanie viditeľné aj v susedných snímkach 5595–5618. |
| PTS 117,322–132,570; 916126–916147 | Pravá zákruta; CTE do +0,930 m, heading do 7,5°. Pulse raw +0,333 → +0,176 → +0,168 → +0,227 → +0,135 pri stabilnom intente/revízii 7. |
| 16. 8. 11:49:38,547; 917634 | Lane CTE −0,843 m, heading −0,8°, R95,8; FF −0,136, raw −0,050, game −0,046. Rovnaká systematická kompenzácia. |
| 11:50:45,493 → 11:50:49,558 | FF +0,581/+0,496/+0,456/+0,411/+0,416; raw +0,509/−0,061/+0,456/−0,451/+0,913. Posledné game −0,095: veľká chyba už v regulátore, nasledovaná oneskorenou odozvou. |
| 11:50:50,146 | Heading 28,6° vyvolá bezpečnostný stop; lokalizácia sa následne stratí. Tento limit zostal zachovaný. |

V poslednom úseku je intent `6893541640d343138518b79e46c8c958`, build
`81a31cbe78fa4505b93ec15b05c8655b`, revízia 8. Posledný uvedený LaneId je
`5962819242599791752:1:0:-:-`, výšková vrstva 65. Mapový dataset
promods-1.59/fingerprint `d6cc7936fce4e902761abb5d` sa nemenil.
Najnovší route-failure z 8. 8. je nedostatočný lane-change approach
(19,177 m dostupných vs. približne 97,9 m potrebných, 4,5 m bočný rozdiel).
Neexistuje nový export dokazujúci chybu topológie pri tejto jazde.

### Koreňové príčiny

1. **Nesprávna kalibrácia fyzikálneho vstupu a nevhodná odozva uzavretej slučky.**
   Hodnota 0,28 rad bola odvodená ako návrhový polomer, nie z prevodu herného
   vstupu na uhol pneumatík. Päť quasi-steady same-LaneId okien dáva ekvivalent
   0,705–0,809 rad pri predpoklade rázvoru 3,8 m, medián 0,788.
   Vybrané riadky: 917595, 917596, 917634, 917638, 917644. Použitá referenčná
   hodnota 0,78 je **odhad konkrétnej konfigurácie**, nie univerzálna SDK konštanta.
   Nesúlad je podopretý logom; presný zisk a oneskorenie musí potvrdiť nové SDK
   meranie. Nezávislý model s týmto ziskom reprodukuje nestabilitu starého kódu.
2. **Kompenzácia návesu menila iba bočný cieľ.** Pri meniacej sa kompenzácii
   nebola korešpondujúca zmena tangenty a curvature. K tomu dlhý pursuit
   horizont meral iné miesto než lokálna poloha. Nový výpočet má jeden lokálny
   rámec a úplnú differential reference, nie proti sebe stojace ciele.
3. **Chyby rozhrania dokázané novými testami:** zmiešané telemetrické snímky,
   starý command pri novom heartbeat/revízii, samostatný emergency scalar,
   latched PAY_TOLL volant, wall-clock skok → záporné dt. Nebolo dokázané,
   že všetky tieto chyby nastali v konkrétnom video-frame; ide o samostatné
   reprodukovateľné príčiny nesprávneho výstupu.

Aktuálny základ už **nemal aktívny opposite_proof/opposite_authorized z 4C/4D**.
Nie je správne vydávať jeho opätovné „odstránenie“ za príčinu tejto opravy.
Staré čisté pomocné funkcie zostávajú pre historické testy, nie ako runtime
regulátory. Nebol nájdený druhý bežný GPS writer. Existujúci Engine articulation
guard je núdzová ochrana, nie bežný paralelný regulátor; v uvedených logoch nebol aktívny.

## Rovnice, jednotky a stav

`e` [m] a `h` [rad] sú LEFT-positive; curvature `k` [1/m], tyre angle
`delta` [rad] a controller `u` [−1,1] sú RIGHT-positive. LaneMatch lateral
sa neguje raz v Map; gameSteer sa neguje raz pri čítaní meranej odozvy.
SCS wheel steering je v **otáčkach**, nie radiánoch: `delta_right = -2*pi*turns`.
Yaw rovnako z rotations/s do rad/s. Primárne SDK definície:
[SCS telemetry truck channels](https://raw.githubusercontent.com/RenCloud/scs-sdk-plugin/master/scs_sdk/include/common/scssdk_telemetry_truck_common_channels.h).

Základné Frenet rovnice:

```text
e_dot = v*sin(h)
h_dot = v*k*cos(h)/(1+k*e) - v*k_vehicle

k_cmd = k_preview*cos(h)/(1+k*e)
        + 2*tan(h)/ell
        + e/(ell^2*cos(h))
delta = atan(3.8*k_cmd)
u = delta/0.78
```

Bez predikcie, pri konštantnej curvature a rýchlosti to dáva presne
`e_ddot + 2*v/ell*e_dot + (v/ell)^2*e = 0`. Znamienka dokazujú testy oboma
smermi, nie iba statická kontrola kladného/záporného výstupu.
Pri platnom yaw sa tie isté pohybové rovnice integrujú dopredu 0,42 s
z **aktuálneho** meraného yaw/v; ide o stateless predikciu, nie históriu povelov.
`ell=max(8 m,3.8 m+v*response_s)`; bez yaw je `ell=8 m+0.84 s*v`.
Modelový preview 0,32+0,10 s nie je presne zmerané oneskorenie z 1 Hz logu.

Náves: `r_target=r_lane+E(s)*left_normal`, `a=1+k*E`,
`h_target=atan2(E',a)`, `k_target=(k-h_target')/sqrt(a²+E'²)`.
Vedenie kabíny používa tento zodpovedajúci heading a curvature, nie iba `e+offset`.
E vychádza z už existujúceho swept-path modelu, rozmerov a šírky pruhu;
meraný trailer CTE nemení jeho stranu. Nie je uložený offset zo starej zákruty,
žiadny integrátor, turn latch, trend timer, moving average ani nový rate limiter.

Ponechané runtime stavy: projekcia/progres kvôli správnemu výskytu LaneId,
existujúce nav/build guardy, Dynamics `command/rate`, engagement/safety stav.
Odstránené aktívne správanie: dlhý pursuit/capture, jeho samostatné steering
clamps, deadband 0,004. Dynamics je jediný fyzický rate/acceleration stupeň.
Existujúce bounds 0,60→0,38 /s a 14→7 /s² sa neuvoľnili. Nie sú to merané
mechanické maximá. Nový arbitrárny jerk limit sa nevymyslel: jerk sa meria;
zmena bounded acceleration na dt dáva diskrétny horný odhad 28/dt.

## Uzavretý model a výsledky

`tests/steering_bench.py`: 20 Hz, deterministický jitter a 110 ms tick, vnútorný
plant krok ≤5 ms, časová command queue 100 ms, presný first-order game actuator
0,32 s (stress 0,50 s), nonlinear bicycle tan(delta), kinematický náves 8 m.
Konštanty plantu sú nezávislé literály, neimportujú sa z regulátora. Testuje sa
aj neznámy zisk 0,70/0,85 a +80 ms odozva pri záťaži, 10–90 km/h s fyzikálne
primeranými polomermi; R18 sa netestuje ako bezpečný pri 90 km/h.

Baseline bol uložený **pred zmenou runtime** v
`tests/fixtures/steering-baseline-e8d5c57.json`. Vstupné pôvodné riadky logu sú
`tests/fixtures/steering-20260816.json`; výsledok je
`tests/fixtures/steering-after-audit.json`. Real-log scenár používa pôvodný
časový priebeh rýchlosti 11:50:30–49 na analytickom R22. **Nie je to presná
rekonštrukcia pôvodnej mapy ani chýbajúcich 20 Hz polôh.**

| Scenár | max CTE pred → po [m] | RMS CTE pred → po [m] | max heading pred → po [°] | ustálenie po [s] |
|---|---:|---:|---:|---:|
| Rovná 90 km/h, počiatočné CTE 0,50 | 8,771 → 0,500 | 1,766 → 0,152 | 66,715 → 0,865 | bez neskorej oscilácie |
| Pravá R250, 90 km/h, náves | 0,910 → 0,146 | 0,694 → 0,065 | 4,655 → 0,791 | 0 |
| Pravá R83, náves | 0,594 → 0,202 | 0,455 → 0,155 | 1,740 → 0,686 | 0 |
| Pravá R35, náves | 0,388 → 0,462 | 0,247 → 0,293 | 2,143 → 1,674 | 1,119 |
| S35, náves | 0,396 → 0,463 | 0,255 → 0,290 | 3,239 → 2,643 | 1,263 |
| Pravá R18, náves | 0,290 → 0,827 | 0,145 → 0,425 | 2,389 → 3,435 | 3,254 |
| Kruhový R25, náves | 0,241 → 0,639 | 0,158 → 0,457 | 1,773 → 2,423 | 2,549 |
| Log-speed R22, lag 0,50 | 8,176 → 0,728 | 1,719 → 0,471 | 52,443 → 3,150 | 2,250 |

CTE kabíny v ostrých návesových zákrutách úmyselne neklesá k nule: ide o
vonkajšiu referenciu pre náves. Tabuľka nezamlčuje jeho zväčšenie. Samostatný
cab-only R18 regression naďalej spĺňa pôvodné max 0,35 m / RMS 0,12 m.
Po výjazde je v každom zo 16 scenárov CTE pod 0,25 m; najhoršie 0,197 m.

Pre log-speed stress: step 0,05643→0,02689; rate 0,57193→0,51367 /s;
acceleration 10,8949→3,6134 /s²; jerk 185,892→134,931 /s³.
Kompletné hodnoty oboch smerov a všetkých scenárov sú v JSON.

Počet opposite samples v **geometricky stálej** zákrute je 0 vo všetkých
nových scenároch. Pôvodná inclusive baseline metrika zahŕňa aj oprávnený výjazd
a S prechod: napr. log stress má 4→1 zmenu znamienka. Nie je korektné vykázať
celú jazdu ako „0 zmien“; zostávajú krátke korekcie pri potvrdenom výjazde.
Klasifikácia sa robí iba z geometrie cez celý derivative/preview footprint,
nikdy podľa toho, či sa nám výsledný volant páči.

Staršie testy boli upravené iba tam, kde predpokladali starý normalizovaný zisk,
ignorovali actuator delay alebo používali Euler/linear-angle plant. Bezlagové
testy teraz explicitne uvádzajú nulový response; model integruje midpoint/tan.
Test 50 % krátkeho zisku používa explicitnú kalibráciu, nie predstieranie jej
znalosti; nezávislý nekalibrovaný rozsah pokrýva nový bench. CTE, confidence,
heading, výška a limity kontinuity neboli zvýšené.

## Záverečný bezpečnostný audit a zostávajúce riziká

Záverečný beh: **493/493 testov, 43,164 s, 0 failures, 0 errors, 0 skipped**.
`compileall` a `git diff --check` prešli. Samostatná read-only AST kontrola
overila jediný aktívny `solve_lateral`, žiadny aktívny pursuit/composition call,
nezmenené chránené navigation/HUD/AR súbory a zhodu SHA256 modelu s reportom.
Zelená testovacia sada neznamená, že otvorená návesová podmienka nižšie prešla.

- NavPath, intent/revision, dataset, LaneLocator hysterézia, HUD/AR/live mapa a
  recorded-route zákaz pri aktívnej GPS sú nezmenené. Pôvodné mapové,
  authority, stale callback/revision a ProMods regresie sa spúšťajú v plnej sade.
- Command packet sa číta raz. Obsahuje output, curvature, revision, intent a
  čas pozorovania/výpočtu. Neplatný alebo starší než existujúci limit 0,5 s
  nejde cez scalar fallback. Krivka nemá uložený stav, ktorý by bolo treba
  resetovať pri novom LaneId. Dynamics si zachová fyzickú polohu volantu.
- Emergency aj toll dostávajú čerstvý rovnaký laterálny povel. Bumpless
  handover prebehne aj pred týmito vetvami. Monotónne dt nemení frekvenciu
  pluginov a záporný skok systémových hodín nemôže zmeniť integráciu.
- **Nevyriešená celá obálka návesu R18:** v širšom pôvodnom 4,7 m teste
  prechádzajú obe nápravy aj šírka tela. Pri 4,5 m najhoršie trailer axle CTE
  1,051 m + polšírka 1,275 m = 2,326 m, teda ~0,076 m za hranou. R35/R83/R25
  taký výsledok nemajú. Statický balanced-offset model nezaručuje clearance
  celého návesu na krátkom výjazde. Spomalenie samo osebe statický offtracking
  neodstráni. Riešením nesmie byť rozšírenie pruhu, filter ani vypínanie ako
  náhrada plánovania. Potrebný je ďalší validovaný swept-envelope planner.
- Prísny globálny zákaz opačného riadenia pre ľubovoľne veľký heading/CTE nie
  je matematickou vlastnosťou tohto zákona. Dokázaný je stabilný pracovný
  rozsah v uzavretých testoch, nie všetky možné stavy hry; fail-closed ostáva.
- Kalibrácia iného ťahača, iné steering nastavenia, fyzikálne extrémne naložený
  náves, trakcia a presné tyre-angle/yaw signály vyžadujú reálnu kontrolu.
  Starý log neumožňuje odvodiť presný jerk ani odozvu medzi každými tickmi.

## Súbory a reprodukcia

Runtime: `core/lateral_controller.py`, `core/navigation/route.py`,
`core/steering_dynamics.py`, `core/sdk/scs_sdk.py`, `core/telemetry.py`,
`core/engine.py`, `core/plugin_manager.py`, `plugins/map/main.py`,
`plugins/autopilot/main.py`.

Testy: `tests/test_steering_system_audit.py`, `tests/steering_bench.py`,
`tests/fixtures/`, a existujúce `test_game_like_control_simulation.py`,
`test_phase4e_closed_loop_controller.py`, `test_phase4_steering_dynamics.py`,
`test_lane_audit_regressions.py`, `test_lane_map_data.py`.
Nástroje: `tools/audit_steering.py`, `tools/run_steering_bench.py`.
Interné nástroje/testy ani tento report runtime allow-list inštalátora nezahŕňa.

```powershell
python -B -m unittest discover -s tests -q
python -B tools/run_steering_bench.py --output docs/steering-audit/recheck.json
python -m compileall -q core plugins tests tools
git diff --check
```

## Plán následnej jazdy — až po výslovnom povolení aktualizácie

1. Bez aktivácie overiť nový `tyre_angles_rad`, `yaw_right_rad_s`, SDK frame
   a znamienka pri manuálnom zatočení oboma smermi. Overiť ekvivalentný zisk
   0,78 na konkrétnom ťahači a steering nastaveniach.
2. Bez premávky: aktivácia s malým bočným posunom, rovina 10/30/60/90 km/h,
   plynulé brzdenie a zastavenie. Zachytiť spoločný čas videa a telemetrie.
3. R83/R35 oboma smermi, rovina po zákrute, S-zákruta a road/prefab hranica.
   Sledovať error voči cieľu, nie iba vizuálnu veľkosť volantu.
4. Kruhový objazd: rôzne výjazdy, až potom náves/zaťaženie. R18 s návesom
   nepovažovať za potvrdený bez merania celého clearance; úzky 4,5 m scenár
   zostáva otvorený.
5. Kontrolovaný authority failure musí vypnúť plyn, bezpečne zastaviť a zapnúť
   výstražné svetlá. Žiadne zníženie confidence/heading limitu.
6. Nakoniec aspoň 50 km s rôznymi rýchlosťami a priebežným porovnaním
   command → out → game → tyre/yaw, CTE, heading, LaneId a revízie.
