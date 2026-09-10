# UltraPilot — Etapa 4A: referenčný bod kabíny a priečne riadenie

Dátum: 2026-09-10. Základ porovnania: `089cd0c`. Žiadny commit, push,
inštalácia, kopírovanie do prevádzkovej aplikácie ani reštart.

## Verdikt a rozsah

**READY FOR CONTROLLED GAME TEST — iba podmienečne:** geometria náprav musí byť
platná a podporovaná a kalibrácia volantu musí zodpovedať konkrétnemu vozidlu.
Prvý test má byť bez návesu a pri nízkej rýchlosti. Implementácia a offline
overenie referenčného bodu sú dokončené; výsledok novej jazdy v ETS2 zatiaľ nie je.

**BLOCKED pre univerzálne schválenie všetkých podvozkov, vysokorýchlostných
odoziev a ostrých manévrov celej súpravy.** Viac pevných zadných náprav alebo
riadenie zadnej nápravy nemajú v tejto zmene preukázaný ekvivalentný model.
Planner obálky návesu nie je súčasťou tejto opravy.

Oprava nemení LanePath, mapové body, šírku pruhu, GPS UID poradie, LaneLocator,
HUD, AR, live mapu, brzdenie ani existujúce bezpečnostné limity. Nepridáva
filter, integrátor, časovú hysteréziu, podržanie volantu ani bočný offset.

## 1. Reálne vstupy a hranice dôkazov

Hlavný log: `C:\Users\PC\AppData\Local\Programs\UltraPilot\ultrapilot.log`,
222 925 846 bajtov, posledný zápis 2026-09-10 15:25:48 miestneho času.
Analyzované exporty sú z tej istej dnešnej jazdy/verzie, nie staré route-failure
exporty iného testu. Časy názvov exportov sú UTC.

| Vstup | Bez návesu | S návesom |
|---|---|---|
| Video | `beznávesu3.mp4` | `snávesom3.mp4` |
| Počet dekódovaných snímok | 11 406 | 9 580 |
| Dĺžka videa | 202,018 s | 170,016 s |
| Aktívna jazda podľa logu | 15:16:21,454–15:19:35,562 | 15:22:54,598–15:25:35,288 |
| Export `steering-replay-…-manual_disable.json` | `20260910T131935.773839Z` | `20260910T132535.496528Z` |
| Počet riadkov exportu | 3 600 | 3 600 |
| Jedinečné aktívne výpočty | 3 335 | 3 101 |
| Jedinečné aktívne pozorované SDK snímky | 2 078 | 1 963 |
| Revision / session | 8 / 1 | 8 / 1 |
| Intent | `706cf7035f444f2d86d08220155beea4` | `be5f3de0c3684cebbd992b182619aa6a` |
| Build | `8935a0dc3b2c4816bfeb26e562bbebc9` | `5c958dd1d2e84d878b08d9515f358495` |
| RMS / max absolútna CTE v pôvodnej jazde | 0,846 / 2,247 m | 0,862 / 2,241 m |

Oba exporty: mapa `promods-1.59`, fingerprint `d6cc7936fce4e902761abb5d`.
SHA-256 exportov:

- bez návesu: `f0fe9ed1a121c2233b4c15805d45e01027e725960cc46ea2fd5df019f6c53677`;
- s návesom: `955b3591fd0cba091772be53a1431004427285c4396eefae38e4f6ceeb8ce471`.

Ring buffer bez návesu začína až 15:16:47,765; prvých približne 26 sekúnd
aktívnej jazdy už neobsahuje. Chýbajúce tick-y sa nedopočítavali. Analýza videí
zahŕňala automatické susedné snímky a vizuálne kontaktné prehľady; optical flow
nie je kalibrovaný uhol volantu. Pri zmene kamery sa nemôže interpretovať ako
otáčanie volantu. Čas vytvorenia video súboru tiež nie je presný začiatok obrazu
kvôli DVR bufferu. Kvantitatívny dôkaz nižšie používa SDK čas výpočtu a rovnaký
zaznamenaný pár poloha–heading, nie odhad uhla z videa.

## 2. Potvrdená chyba modelu

Pôvodný regulátor aj plant v benchmarku predpokladali, že pozorovaná poloha je
bod zadnej nápravy bicyklového modelu. Reálna poloha zo SDK sa však pri zatáčaní
pohybuje aj priečne voči osi kabíny. Je to správanie bodu pred zadnou nápravou,
nie správanie zadnej nápravy bez bočného sklzu.

Z dvojíc skutočne zaznamenaných pozícií vzdialených 0,18–0,35 s sa počíta
priečna rýchlosť a zmena orientácie. Pre bod vo vzdialenosti `a` pred nápravou
platí `v_left = a * yaw_left`. Vyhodnocujú sa iba páry rovnakej LaneId,
intent/build/revision/session/map/fingerprint, s rýchlosťou nad 3 m/s a
nenulovým zatáčaním. Žiadna interpolácia tickov.

| Odhad efektívnej vzdialenosti pred nápravou | Bez návesu | S návesom |
|---|---:|---:|
| Počet intervalov | 1 422 | 1 347 |
| Medián všetkých intervalov | 2,136 m | 2,040 m |
| 10.–90. percentil | 2,053–2,206 m | 1,957–2,098 m |
| Medián ľavých zákrut | 2,131 m | 2,054 m |
| Medián pravých zákrut | 2,126 m | 2,067 m |

Toto je **kinematický odhad**, nie univerzálna kalibrácia. Sklz pneumatík,
zaťaženie a priestorová orientácia môžu odhad ovplyvniť. Starý replay neobsahoval
statické polohy náprav, preto ich presné hodnoty v starej jazde nepredstierame.
Nový runtime získava geometriu priamo zo SDK; hodnotu 2,1 m nepoužíva naslepo.

### Konkrétny časový dôkaz

15:18:47,440, SDK frame `207108382`, road UID `5962819247976890548`, smer 1,
lane index 0, revision 8:

| Člen | Pôvodná hodnota |
|---|---:|
| LaneMatch CTE, kladná doprava | +2,246763 m |
| CTE regulátora, kladná doľava | −2,246763 m |
| Orientácia kabíny | −2,082005911 rad |
| Lokálna tangenta | −2,189386601 rad |
| Podpísaný rozdiel kabína–tangenta | +0,107380690 rad ≈ 6,15° |
| Lokálne zakrivenie | +0,049517430 1/m |
| Preview zakrivenie | +0,045140748 1/m |
| Feed-forward | +0,270845 |
| Heading feedback | +0,157925 |
| CTE feedback | −0,159857 |
| Výsledný raw povel | +0,268912 |

Pri správne vycentrovanom prednom referenčnom bode potrebuje kabína v takej
zákrute prirodzený rozdiel oproti tangente približne `asin(2.1*k) ≈ 6°`.
Starý regulátor považoval tento potrebný rozdiel za chybu smeru. K základnému
natočeniu pridával ďalšie zatočenie. Až veľká opačne podpísaná CTE ho vyvážila.
Preto mohol byť volant približne ustálený, ale kabína bola systematicky pri
vnútornej hrane zákruty. Ide o nesprávny fyzikálny referenčný bod, nie dôkaz,
že treba posunúť mapovú strednicu alebo pridať filter.

Nasledujúci skutočný SDK frame `207358372` má polohu
`(37165.0000305, 58740.5058594)` a heading `−2.157185200` rad, predchádzajúci
`(37163.8340149, 58739.6039886)`. Táto konkrétna dvojica aj pôvodný výstup sú
uložené ako reprodukčné hodnoty v novom teste.

Uzavretá reprodukcia na analytickej R22, 6 m/s, gain 0,70, rovnaký plant:

| Predpoklad regulátora | Max CTE | RMS CTE |
|---|---:|---:|
| Starý: kabína sa považuje za zadnú nápravu | 2,030899 m | 1,113248 m |
| Nový: pozorovaný bod 2,1 m pred nápravou | 0,323510 m | 0,090095 m |

Test najskôr zlyhal na starom kóde vľavo aj vpravo. Limit 0,70 m sa nezmenil.
Tieto čísla sú výsledky simulácie, **nie nová jazda v ETS2**.

## 3. Nový výpočet a jednotky

- `e` [m]: odchýlka pozorovaného bodu doľava od pruhu.
- `h` [rad]: podpísaný rozdiel orientácie kabíny od lokálnej tangenty, doľava kladný.
- `k`, `kp` [1/m]: lokálne a preview zakrivenie pruhu, doprava kladné.
- `a` [m]: pozorovaný bod pred zadnou nápravou; `L` [m]: rázvor.
- `ell = max(8 m, L + |v|*response_s)`: priestorová dĺžka feedbacku.

Kinematika pozorovaného bodu:

```text
e_dot = v * (sin(h) - a * k_vehicle * cos(h))
s_dot = v * (cos(h) + a * k_vehicle * sin(h)) / (1 + k*e)
h_dot = k*s_dot - v*k_vehicle
```

Rovnovážna orientácia kabíny v lokálnom kruhu a kompozícia regulátora:

```text
d         = 1 + k*e
h_ref     = asin(a*k/d)
h_control = wrap(h - h_ref)
k_ff      = kp*cos(h_control) / sqrt(d² - (a*kp)²)
k_h       = 2*1.10*tan(h_control) / ell
k_e       = 0.85*e / (ell²*cos(h_control))
k_demand  = k_ff + k_h + k_e
tyre_rad  = atan(L*k_demand)
command   = tyre_rad / configured_lock_rad
```

Pri `a=0` je to pôvodný model zadnej nápravy. Konštantný kruh má správnu
geometrickú rovnováhu; pre meniace sa zakrivenie ide o lokálny sledovací
regulátor overený simuláciou, nie formálny dôkaz pre každú možnú cestu/plant.
Nedosiahnuteľný referenčný kruh, neplatné čísla a nepreukázaná geometria sú
odmietnuté, nie skryté saturáciou vstupu do `asin`.

Znamienka zostali: LaneMatch CTE sa v Map neguje práve raz; heading sa počíta
z orientácie kabíny a tangenty; pravé zakrivenie/uhol/povel sú kladné. SDK
gameSteer a wheel-steering turns sa konvertujú existujúcimi negáciami na
hranici telemetrie. Nová geometria nepridáva negáciu výstupného volantu.

## 4. Dátový tok a bezpečnosť

```text
SDK: wheel count + local X/Z + steerable flags
  -> Telemetry.referenceGeometry
  -> jeden Engine vehicle_envelope_snapshot spolu s polohou/heading/SDK časom
  -> Map -> Route -> jediný lateral_controller
  -> rovnaký revision/intent/build-bound command packet
  -> jediný existujúci SteeringDynamics -> Engine -> SCSControls
```

Overené SDK offsety: wheel count 80, steerable flags 1500, wheel X 1676,
wheel Z 1804. Sú pokryté binárnym ABI testom. Lokálne +Z smeruje dozadu:
`a=rear_axle_z`, `L=rear_axle_z-front_axle_z`.

Podporovaný je jednoznačný predný riadený a zadný pevný nápravový pár
(prípadné zdvojené kolesá v rovnakej rovine). Každá skupina musí mať kolesá
na oboch stranách a preukázanú spoločnú nápravovú rovinu. Neplatný rozsah,
chýbajúce dáta, asymetria, tandem s viacerými polohami Z alebo zadné riadenie
zostávajú fail-closed. **To môže zablokovať zapnutie na 6x2/6x4/8x4.**
Bez dôkazu rozdelenia zaťaženia sa neháda ich ekvivalentná zadná náprava.

Runtime vždy odovzdáva geometriu zo SDK. Chýbajúca geometria nespadne do
matematického defaultu `a=0`. Ten ostáva iba pre samostatné staré geometrické
testy/nástroje. Zlyhanie sa prenesie presným `control_failure`; moderný paket
nespadne na starý scalar `nav_steering`.

Geometria sa neuchováva v regulátore medzi tickmi. Zmena vozidla/revízie
nezdedí skrytý stav. Snapshot, route points a LaneId sa neprepisujú.
Test preukázanej road→prefab hranice potvrdzuje totožný povel pri zmene
vlastníctva spoločného segmentu. Existujúce stale/revision/session/map,
protismerné, výškové a confidence testy ostali aktívne.

Tyre/yaw sú stále iba diagnostika, nie druhá rýchla spätná väzba. Stály kruh
s nezmeneným e/h/k dáva presne rovnaký raw povel aj pri 100 rôznych hodnotách
oneskorenej tyre telemetrie. To nie je dôkaz odstránenia všetkých možných
vizuálnych mikropohybov v hre; tie sa znovu skontrolujú v novom dense exporte.

## 5. Rovnaký nezávislý plant pred/po

20 Hz riadenie, integrácia najviac 5 ms, časové oneskorenie, jitter vrátane
oneskoreného ticku, šum CTE/heading, kabína alebo zjednodušený článok návesu.
Geometria a parametre plantu sa neimportujú z konštánt regulátora.
Pozorovaná poloha plantu je 2,1 m pred zadnou nápravou. Nominal gain 0,78 je
explicitne rovnaký na oboch stranách tohto benchmarku, nie runtime tvrdenie
o univerzálnej kalibrácii; vyššie uvedená R22 reprodukcia používa 0,70.

Tabuľka uvádza horší výsledok ľavého/pravého variantu, kde existujú oba.

| Scenár | Rýchlosť | Max CTE pred → po | RMS CTE pred → po |
|---|---:|---:|---:|
| R18 | 19,4 km/h | 2,282 → 0,386 m | 1,069 → 0,122 m |
| R35 | 26,6 km/h | 1,252 → 0,218 m | 0,759 → 0,059 m |
| R83 | 43,2 km/h | 0,615 → 0,152 m | 0,470 → 0,033 m |
| R250 | 90 km/h | 0,342 → 0,150 m | 0,296 → 0,031 m |
| S35 | 26,6 km/h | 1,209 → 0,414 m | 0,702 → 0,091 m |
| Kruhový objazd R25 | 21,6 km/h | 1,789 → 0,281 m | 1,207 → 0,075 m |
| R22, log-speed, odozva 0,32 s | časový profil | 2,024 → 0,353 m | 1,205 → 0,114 m |
| R22, log-speed, odozva 0,50 s | časový profil | 2,007 → 0,275 m | 1,202 → 0,093 m |

Log-speed benchmark používa existujúci rýchlostný profil fixture zo staršieho
logu na analytickej geometrii. Nie je to rekonštrukcia chýbajúcej LanePath
septembrovej jazdy a nemieša sa s ňou pri dokazovaní konkrétnej chyby.

| Metrika po oprave | R18 | S35 | R25 | R22 / 0,32 s |
|---|---:|---:|---:|---:|
| Max rozdiel kabína–tangenta | 9,10° | 6,12° | 6,52° | 8,17° |
| Max krok výstupu [input] | 0,02050 | 0,01559 | 0,01234 | 0,01600 |
| Max rýchlosť volantu [input/s] | 0,405 | 0,260 | 0,255 | 0,323 |
| Max acceleration [input/s²] | 9,865 | 4,742 | 7,956 | 8,179 |
| Max diskrétny jerk [input/s³] | 329,99 | 166,88 | 274,03 | 251,51 |
| Ustálenie po výjazde podľa starej metriky | 3,65 s | 2,31 s | 2,77 s | 2,80 s |
| Nežiaduce zmeny znamienka v monotónnej zákrute | 0 | 0 | 0 | 0 |

Rozdiel kabína–tangenta už nemožno celý nazývať chybou: časť je fyzicky
potrebný `h_ref`. Diagnostika preto uchováva pôvodný h aj nový
`body_reference_heading_rad` a `body_tracking_error_rad`. Jerk sa meria,
nepribudol ďalší jerk limiter.

### Roviny bez regresie

Všetky metriky štyroch origin-aware rovín sú pred/po presne rovnaké.
Pôvodný 16-scenárový benchmark s referenciou zadnej nápravy má tiež nulový
rozdiel max CTE po tejto zmene.

| Rýchlosť | RMS CTE pred = po | Max CTE pred = po | CTE po 15 m za poslednou sekciou |
|---|---:|---:|---:|
| 10 km/h | 0,09762 m | 0,500 m | 0,00195 m |
| 30 km/h | 0,09956 m | 0,500 m | 0,00365 m |
| 60 km/h | 0,12029 m | 0,500 m | 0,00370 m |
| 90 km/h | 0,13992 m | 0,500 m | 0,00929 m |

Max 0,5 m je zámerná počiatočná odchýlka testu, nie vzniknutá strata pruhu.

### Citlivostná matica — nie všetko je schválené

| Súbor testov | Výsledok a obmedzenie |
|---|---|
| 198 scenárov, plant gain 0,60–0,95, odozva 0,25–0,60 s, kontrolér stále 0,78 | Pred: 18 prípadov prekročilo 2,4 m. Po: 0; všetky dokončili trasu a 0 monotónnych opačných povelov. Max CTE však zostáva 1,483 m. Toto **nespĺňa** prísny cieľ centrovania. |
| 198 scenárov, gain regulátora explicitne kalibrovaný k plantu | Všetky dokončené; 0 strát a 0 monotónnych opačných povelov; max CTE 0,500 m vrátane počiatočnej odchýlky rovín. |
| Pôvodná metrika `steady_cte` | Začína už 15 m po zákrute, pri 90 km/h po 0,6 s. V 12 kalibrovaných prípadoch presiahne 0,25 m počas prechodového deja. Hodnoty ostali nezmenené a viditeľné. |
| Nové testy ustálenia | Celý koniec trasy **po 4 sekundách** od výjazdu: menej než 0,25 m, pri kombinovanej odozve plantu do 0,60 s; nie iba posledná náhodná vzorka. |
| Mimoriadne pomalý plant 0,68 s, 90 km/h, R250 | Max CTE 0,431 m, ale aj po 8 s približne 0,290 m. Komfort/ustálenie **neprešlo** cieľom 0,25 m. Samostatný diagnostický test tento otvorený problém explicitne zachováva; nie je započítaný ako úspešná výkonnostná akceptácia. |

Nový test pôvodne neúmyselne sčítal 0,60 s a prídavných 0,08 s za zaťaženie.
Podporovaný sweep teraz explicitne používa 0,52+0,08=0,60 s; samostatný
0,68 s test sa nevymazal. Nezmenila sa žiadna tolerancia starého testu.
Kalibrácia zostáva statická, nebol pridaný online adaptívny regulátor.

## 6. Zmenené súbory a staré testy

| Súbory | Zmena |
|---|---|
| `core/vehicle_geometry.py` | Nové overenie statickej geometrie náprav a referenčného bodu. |
| `core/sdk/scs_sdk.py`, `core/telemetry.py` | Čítanie count/X; normalizácia geometrie spolu s existujúcim Z/steerable. |
| `core/engine.py` | Jedno nové pole v existujúcom atómovom vehicle snapshote. |
| `core/lateral_controller.py` | Fyzikálne správna lokálna rovnováha pozorovaného bodu, rázvor zo SDK. |
| `core/navigation/route.py` | Explicitný vstup geometrie, validácia a diagnostika; bez úpravy trasy. |
| `plugins/map/main.py` | Prenos geometrie rovnakého pozorovania do existujúceho regulátora. |
| `plugins/autopilot/main.py` | Presný dôvod odmietnutia; nové dense diagnostické polia. |
| `tests/test_stage4a_reference_point.py` | 13 nových testov vrátane reálnych hodnôt, closed-loop, ABI, hranice LaneId a otvoreného extrémneho stress prípadu. |
| `tests/steering_bench.py` | Voliteľný fyzický pozorovaný bod. Default `a=0` a pôvodné limity/metriky zachované. |
| `tests/test_lane_authority_integration.py` | Geometry fixture spoločného buildera a dvoch recorded-route testov. |
| `tests/test_real_20260906_regressions.py` | Geometry fixture testu neblokujúceho publikovania paketov. |
| `tests/test_steering_replay.py` | Geometry fixture + silnejší assert platnosti paketu. |
| `tools/run_steering_bench.py` | Historický Route musí načítať aj historický solver. Podporuje aj e8d5c57, ktorý samostatný modul ešte nemal. |
| `tools/run_stage4a_bench.py`, `tools/audit_reference_point.py` | Reprodukovateľné before/after porovnanie a read-only audit dense pozícií. |
| `STAGE4A_AUDIT.md` | Tento report. |

Presné zmeny starých testov:

- `build_map_plugin`: doplnená explicitná geometria syntetického 4x2. Inak
  by test normálneho riadenia skúšal novú vetvu „chýba povinná geometria“.
- `test_recorded_route_runs_only_after_explicit_load_without_gps` a
  `test_recorded_route_does_not_resume_after_gps_is_removed`: to isté; všetky
  pôvodné očakávania a ochrana proti recorded fallbacku ostali.
- `test_blocked_presentation_save_cannot_stop_fresh_lateral_packets`:
  doplnená geometria do vlastného snapshotu; časový limit 0,1 s ostal.
- `test_map_publishes_one_bound_packet_without_changing_steering`:
  doplnená geometria a `assertTrue(authority_valid)`, aby nemohol prejsť
  porovnaním dvoch núdzových núl.
- Žiadne pôvodné znamienko, očakávaný povel, confidence, CTE limit, fyzikálny
  gain ani odozva v starých akceptačných testoch neboli zmäkčené.

## 7. Overenie

- Nové cielené testy: **13 prešlo, 27 podtestov prešlo**.
- Celá sada: **595 testov prešlo, 1 293 podtestov prešlo**; 56,15 s.
- Pôvodný benchmark: 16 scenárov pred/po.
- Origin-aware benchmark: 16 nominálnych + 198 nekalibrovaných scenárov pred/po;
  samostatne 198 kalibrovaných scenárov po oprave. Výkonnostné obmedzenia vyššie.
- `compileall core plugins tests tools`: prešlo.
- `git diff --check`: prešlo; Git iba upozorňuje na pracovné LF/CRLF konce.

Prvý pytest beh v sandboxe narazil na Windows oprávnenia dočasného priečinka
pri upratovaní. Úspešný finálny beh prebehol s povoleným spustením mimo
sandboxu, stále s dočasnými súbormi v repozitári. Staré neprístupné cache
priečinky neboli odstraňované. Testy sa zbierali z `tests/`, nie z cache.

Reprodukčné príkazy (z pracovného repozitára):

```powershell
python -B -m pytest tests -q -p no:cacheprovider --basetemp=docs/steering-audit/pytest-stage4a-complete
python -m compileall -q core plugins tests tools
git diff --check
python -B tools/run_stage4a_bench.py --ref 089cd0c --matrix --output docs/steering-audit/stage4a-origin-before.json
python -B tools/run_stage4a_bench.py --matrix --output docs/steering-audit/stage4a-origin-after.json
python -B tools/run_stage4a_bench.py --matrix --matched-calibration --output docs/steering-audit/stage4a-origin-complete.json
```

Výstupy sú v `docs/steering-audit/` (priečinok je existujúcim `.gitignore`
ignorovaný); nástroje, testy a tento report sú normálne zdrojové súbory.

## 8. Čo ešte nie je vyriešené

1. Reálne potvrdenie novej opravy v ETS2. Staré video nie je dôkaz novej jazdy.
2. Podvozky s viacerými efektívnymi zadnými nápravami alebo zadným riadením.
   Ich automatické odmietnutie je dátové obmedzenie tejto implementácie.
3. Výrazne iný input gain/odozva než identifikované nastavenie. Najmä extrémny
   0,68 s plant pri 90 km/h nemá splnený komfortný limit ustálenia.
4. Náves: matematické centrovanie kabíny nezaručuje rezervu celej súpravy.
   Reálny trailer export mal zápornú odhadovanú clearance približne −0,915 m;
   offset zostal 0 a žiadna manévrovacia referencia nebola autorizovaná.
   R18/4,5 m sa **neoznačuje za bezpečný prejazd návesu**. Predchádzajúci
   známy presah obálky nemožno touto opravou vyhlásiť za odstránený.
5. Merateľný sklz, náklon, nerovnosti a nepresnosť lokálnej mapovej geometrie
   môžu vyžadovať ďalšie samostatne dokázané opravy. Neodvodzuje sa bočný
   offset ani plošná úprava datasetu zo samotnej chyby riadenia.

## 9. Prvý kontrolovaný test

1. Až po používateľom vykonanej aktualizácii skontrolovať novú dense diagnostiku:
   `reference_geometry.valid=true`, `source=sdk_wheel_positions`, rozumný
   `wheelbase_m` a `reference_ahead_m`; kalibrácia súčasného testovaného setupu
   má zostať `lock_rad=0.700`. Číslo sa nepoužije automaticky pre iný setup.
2. Bez návesu, prázdny bezpečný úsek: rovina 20–30 km/h, potom dlhá ľavá/pravá
   zákruta. Sledovať centrovanie kabíny aj korekcie počas už ustálenej zákruty.
3. Bežná S-zákruta a výjazd na rovinu. Na začiatku nie ostré 90°/R18 manévre.
4. Až po úspechu 40–60 km/h. Test 90 km/h odložiť, kým nová telemetria nepotvrdí
   vhodnú odozvu daného vozidla. Rýchlosť vždy prispôsobiť zákrute.
5. Manuálne vypnúť autopilot a skontrolovať nový `manual_disable` replay.
   Pre test s návesom vytvoriť samostatný export; začať bežnou širšou cestou.
6. Pri raste odchýlky, nečakaných pohyboch alebo neplatnej geometrii test ukončiť
   ručne. Neriešiť to zvýšením confidence/heading/stale limitov.

Porovnávať v rovnakom calculation SDK frame: `lane_cte_m` (doprava kladné),
`predicted_frenet_cte_m` (doľava kladné), pôvodný podpísaný heading,
`body_reference_heading_rad`, `body_tracking_error_rad`, `local_k_per_m`,
`preview_k_per_m`, FF/heading/CTE príspevky, `controller_steer_raw`, `steer_out`,
`game_steer_right`, tyre angles, yaw, packet age/frame lag, LaneId a úplnú
identitu trajektórie. Nové statické rozmery sa kontrolujú proti údajom o
podvozku, nie online prispôsobovaním regulátora podľa chyby.
