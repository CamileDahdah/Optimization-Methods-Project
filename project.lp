\ Model final_project
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  7 empty_0_AB + 10 available_cargo_0_AB + 3 empty_0_AC
   + 10 available_cargo_0_AC + 7 empty_0_BA + 10 available_cargo_0_BA
   + 6 empty_0_BC + 10 available_cargo_0_BC + 3 empty_0_CA
   + 10 available_cargo_0_CA + 6 empty_0_CB + 10 available_cargo_0_CB
   + 7 empty_1_AB + 10 available_cargo_1_AB + 3 empty_1_AC
   + 10 available_cargo_1_AC + 7 empty_1_BA + 10 available_cargo_1_BA
   + 6 empty_1_BC + 10 available_cargo_1_BC + 3 empty_1_CA
   + 10 available_cargo_1_CA + 6 empty_1_CB + 10 available_cargo_1_CB
   + 7 empty_2_AB + 10 available_cargo_2_AB + 3 empty_2_AC
   + 10 available_cargo_2_AC + 7 empty_2_BA + 10 available_cargo_2_BA
   + 6 empty_2_BC + 10 available_cargo_2_BC + 3 empty_2_CA
   + 10 available_cargo_2_CA + 6 empty_2_CB + 10 available_cargo_2_CB
   + 7 empty_3_AB + 10 available_cargo_3_AB + 3 empty_3_AC
   + 10 available_cargo_3_AC + 7 empty_3_BA + 10 available_cargo_3_BA
   + 6 empty_3_BC + 10 available_cargo_3_BC + 3 empty_3_CA
   + 10 available_cargo_3_CA + 6 empty_3_CB + 10 available_cargo_3_CB
   + 7 empty_4_AB + 10 available_cargo_4_AB + 3 empty_4_AC
   + 10 available_cargo_4_AC + 7 empty_4_BA + 10 available_cargo_4_BA
   + 6 empty_4_BC + 10 available_cargo_4_BC + 3 empty_4_CA
   + 10 available_cargo_4_CA + 6 empty_4_CB + 10 available_cargo_4_CB
Subject To
 t_1_4: flight_4_AC + empty_4_AC + flight_4_BC + empty_4_BC + stay_4_C
   = 1200
 t_2_0_A: flight_0_BA + empty_0_BA + flight_0_CA + empty_0_CA - flight_1_AB
   - empty_1_AB - flight_1_AC - empty_1_AC + stay_0_A - stay_1_A = 0
 t_2_0_B: flight_0_AB + empty_0_AB + flight_0_CB + empty_0_CB - flight_1_BA
   - empty_1_BA - flight_1_BC - empty_1_BC + stay_0_B - stay_1_B = 0
 t_2_0_C: flight_0_AC + empty_0_AC + flight_0_BC + empty_0_BC - flight_1_CA
   - empty_1_CA - flight_1_CB - empty_1_CB + stay_0_C - stay_1_C = 0
 t_2_1_A: flight_1_BA + empty_1_BA + flight_1_CA + empty_1_CA - flight_2_AB
   - empty_2_AB - flight_2_AC - empty_2_AC + stay_1_A - stay_2_A = 0
 t_2_1_B: flight_1_AB + empty_1_AB + flight_1_CB + empty_1_CB - flight_2_BA
   - empty_2_BA - flight_2_BC - empty_2_BC + stay_1_B - stay_2_B = 0
 t_2_1_C: flight_1_AC + empty_1_AC + flight_1_BC + empty_1_BC - flight_2_CA
   - empty_2_CA - flight_2_CB - empty_2_CB + stay_1_C - stay_2_C = 0
 t_2_2_A: flight_2_BA + empty_2_BA + flight_2_CA + empty_2_CA - flight_3_AB
   - empty_3_AB - flight_3_AC - empty_3_AC + stay_2_A - stay_3_A = 0
 t_2_2_B: flight_2_AB + empty_2_AB + flight_2_CB + empty_2_CB - flight_3_BA
   - empty_3_BA - flight_3_BC - empty_3_BC + stay_2_B - stay_3_B = 0
 t_2_2_C: flight_2_AC + empty_2_AC + flight_2_BC + empty_2_BC - flight_3_CA
   - empty_3_CA - flight_3_CB - empty_3_CB + stay_2_C - stay_3_C = 0
 t_2_3_A: flight_3_BA + empty_3_BA + flight_3_CA + empty_3_CA - flight_4_AB
   - empty_4_AB - flight_4_AC - empty_4_AC + stay_3_A - stay_4_A = 0
 t_2_3_B: flight_3_AB + empty_3_AB + flight_3_CB + empty_3_CB - flight_4_BA
   - empty_4_BA - flight_4_BC - empty_4_BC + stay_3_B - stay_4_B = 0
 t_2_3_C: flight_3_AC + empty_3_AC + flight_3_BC + empty_3_BC - flight_4_CA
   - empty_4_CA - flight_4_CB - empty_4_CB + stay_3_C - stay_4_C = 0
 t_2_4_A: - flight_0_AB - empty_0_AB - flight_0_AC - empty_0_AC
   + flight_4_BA + empty_4_BA + flight_4_CA + empty_4_CA - stay_0_A
   + stay_4_A = 0
 t_2_4_B: - flight_0_BA - empty_0_BA - flight_0_BC - empty_0_BC
   + flight_4_AB + empty_4_AB + flight_4_CB + empty_4_CB - stay_0_B
   + stay_4_B = 0
 t_2_4_C: - flight_0_CA - empty_0_CA - flight_0_CB - empty_0_CB
   + flight_4_AC + empty_4_AC + flight_4_BC + empty_4_BC - stay_0_C
   + stay_4_C = 0
 t_3_0_A: flight_0_AB + flight_0_AC + available_cargo_0_AC
   - available_cargo_4_AB - available_cargo_4_AC = 150
 t_3_0_B: flight_0_BA + flight_0_BC + available_cargo_0_BC
   - available_cargo_4_BA - available_cargo_4_BC = 50
 t_3_0_C: flight_0_CA + flight_0_CB + available_cargo_0_CB
   - available_cargo_4_CA - available_cargo_4_CB = 440
 t_3_1_A: - available_cargo_0_AB - available_cargo_0_AC + flight_1_AB
   + flight_1_AC + available_cargo_1_AC = 250
 t_3_1_B: - available_cargo_0_BA - available_cargo_0_BC + flight_1_BA
   + flight_1_BC + available_cargo_1_BC = 50
 t_3_1_C: - available_cargo_0_CA - available_cargo_0_CB + flight_1_CA
   + flight_1_CB + available_cargo_1_CB = 240
 t_3_2_A: - available_cargo_1_AB - available_cargo_1_AC + flight_2_AB
   + flight_2_AC + available_cargo_2_AC = 150
 t_3_2_B: - available_cargo_1_BA - available_cargo_1_BC + flight_2_BA
   + flight_2_BC + available_cargo_2_BC = 50
 t_3_2_C: - available_cargo_1_CA - available_cargo_1_CB + flight_2_CA
   + flight_2_CB + available_cargo_2_CB = 340
 t_3_3_A: - available_cargo_2_AB - available_cargo_2_AC + flight_3_AB
   + flight_3_AC + available_cargo_3_AC = 450
 t_3_3_B: - available_cargo_2_BA - available_cargo_2_BC + flight_3_BA
   + flight_3_BC + available_cargo_3_BC = 50
 t_3_3_C: - available_cargo_2_CA - available_cargo_2_CB + flight_3_CA
   + flight_3_CB + available_cargo_3_CB = 240
 t_3_4_A: - available_cargo_3_AB - available_cargo_3_AC + flight_4_AB
   + flight_4_AC + available_cargo_4_AC = 350
 t_3_4_B: - available_cargo_3_BA - available_cargo_3_BC + flight_4_BA
   + flight_4_BC + available_cargo_4_BC = 50
 t_3_4_C: - available_cargo_3_CA - available_cargo_3_CB + flight_4_CA
   + flight_4_CB + available_cargo_4_CB = 440
 t_4_0_CB: <= 0
 t_4_0_AB: flight_0_AB - available_cargo_4_AB <= 100
 t_4_0_AC: flight_0_AC - available_cargo_4_AC <= 50
 t_4_0_BA: flight_0_BA - available_cargo_4_BA <= 25
 t_4_0_BA: <= 0
 t_4_0_BC: flight_0_BC - available_cargo_4_BC <= 25
 t_4_0_CA: flight_0_CA - available_cargo_4_CA <= 40
 t_4_0_CB: flight_0_CB - available_cargo_4_CB <= 400
 t_4_0_CB: <= 0
 t_4_1_CB: <= 0
 t_4_1_AB: - available_cargo_0_AB + flight_1_AB <= 200
 t_4_1_AC: - available_cargo_0_AC + flight_1_AC <= 50
 t_4_1_BA: - available_cargo_0_BA + flight_1_BA <= 25
 t_4_1_BA: <= 0
 t_4_1_BC: - available_cargo_0_BC + flight_1_BC <= 25
 t_4_1_CA: - available_cargo_0_CA + flight_1_CA <= 40
 t_4_1_CB: - available_cargo_0_CB + flight_1_CB <= 200
 t_4_1_CB: <= 0
 t_4_2_CB: <= 0
 t_4_2_AB: - available_cargo_1_AB + flight_2_AB <= 100
 t_4_2_AC: - available_cargo_1_AC + flight_2_AC <= 50
 t_4_2_BA: - available_cargo_1_BA + flight_2_BA <= 25
 t_4_2_BA: <= 0
 t_4_2_BC: - available_cargo_1_BC + flight_2_BC <= 25
 t_4_2_CA: - available_cargo_1_CA + flight_2_CA <= 40
 t_4_2_CB: - available_cargo_1_CB + flight_2_CB <= 300
 t_4_2_CB: <= 0
 t_4_3_CB: <= 0
 t_4_3_AB: - available_cargo_2_AB + flight_3_AB <= 400
 t_4_3_AC: - available_cargo_2_AC + flight_3_AC <= 50
 t_4_3_BA: - available_cargo_2_BA + flight_3_BA <= 25
 t_4_3_BA: <= 0
 t_4_3_BC: - available_cargo_2_BC + flight_3_BC <= 25
 t_4_3_CA: - available_cargo_2_CA + flight_3_CA <= 40
 t_4_3_CB: - available_cargo_2_CB + flight_3_CB <= 200
 t_4_3_CB: <= 0
 t_4_4_CB: <= 0
 t_4_4_AB: - available_cargo_3_AB + flight_4_AB <= 300
 t_4_4_AC: - available_cargo_3_AC + flight_4_AC <= 50
 t_4_4_BA: - available_cargo_3_BA + flight_4_BA <= 25
 t_4_4_BA: <= 0
 t_4_4_BC: - available_cargo_3_BC + flight_4_BC <= 25
 t_4_4_CA: - available_cargo_3_CA + flight_4_CA <= 40
 t_4_4_CB: - available_cargo_3_CB + flight_4_CB <= 400
 t_4_4_CB: <= 0
Bounds
End
