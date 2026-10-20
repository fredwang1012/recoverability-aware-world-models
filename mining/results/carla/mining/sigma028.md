# Mining claims re-flagged at the max observed repeat spread (0.028)

Task 5 used sigma = 0.01666 (observed corridor and rc03 repeats). Task 8 later observed a spread of 0.028 (RC13 scene 14/20/0.2). This file recomputes the near-threshold sets from the local records at both values. Only scenes that already have D4 repeats can be checked for a changed label; the others are listed as NOT REPEATED and nothing is assumed about them. Margin units: clearance metres / q_clear = 5.0, so 0.028 = 0.14 m.

### corridor

Pool: 677 distinct scenes (records + pair members), label-grade V = max_j min_w R_raw.

| sigma (margin units) | scenes with finite V, abs(V - eps) <= sigma | of which with D4 repeats | of which NOT REPEATED | repeated rows UNRESOLVED (label side/kind changed) |
|---|---|---|---|---|
| 0.01666 | 16 | 16 | 0 | 0 |
| 0.028 | 21 | 16 | 5 | 0 |

D4 rows (all, including the extended set): 32; rows with spread > 0: 9; max spread 0.00769; rows UNRESOLVED over all repeats: 0.

near_boundary (summary.md definition, abs(V_search) < 0.3): 523 of 722 scene records; the window is about 10 times the spread, so it is insensitive to sigma; 24 records have abs(V_search) <= 0.028 (V_search is the finite search grade, so this is only an indicator).

label_vs_anchor_agree excluding scenes with abs(analytic margin) <= 0.083 m (= 0.01666 x q_clear 5): 668/717 = 0.932; scenes excluded as inside the noise band: 5
label_vs_anchor_agree excluding scenes with abs(analytic margin) <= 0.140 m (= 0.028 x q_clear 5): 667/711 = 0.938; scenes excluded as inside the noise band: 11
label_vs_anchor_agree over all 722 R/U-labelled scene records: 671/722 = 0.929.

### rc03

Pool: 639 distinct scenes (records + pair members), label-grade V = max_j min_w R_raw.

| sigma (margin units) | scenes with finite V, abs(V - eps) <= sigma | of which with D4 repeats | of which NOT REPEATED | repeated rows UNRESOLVED (label side/kind changed) |
|---|---|---|---|---|
| 0.01666 | 63 | 63 | 0 | 0 |
| 0.028 | 88 | 68 | 20 | 0 |

D4 rows (all, including the extended set): 77; rows with spread > 0: 14; max spread 0.01666; rows UNRESOLVED over all repeats: 0.

near_boundary (summary.md definition, abs(V_search) < 0.3): 477 of 689 scene records; the window is about 10 times the spread, so it is insensitive to sigma; 96 records have abs(V_search) <= 0.028 (V_search is the finite search grade, so this is only an indicator).

label_vs_anchor_agree: no analytic anchor for this family (n/a).
