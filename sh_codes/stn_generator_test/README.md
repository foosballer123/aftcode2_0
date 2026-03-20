
This folder contains a file generation scheme for automating the creation of a series of simulated tests to work in tandem with `MPC_Solver_Y.py` found in `aftcode2_0/scripts/solvers`.

**Note:** Since creating this, I have modified the file structure of `aftcode2_0` which might cause errors with running the shell scripts in this folder. If this is the case, you might need to modify the folder pathways in any of these shell scripts; `generator_template.sh`, `get_params.sh`, `meta_generator.sh`, `run_generators.sh`, `run_trials.sh`, or `template.sh`.

**To generate tests:**

  1. Open `schemes.txt` and make sure that all of the trials you wish to run are present. If not, modify this file.
  2. Open a terminal in this file directory (`\stn_generator_test`).
  3. Run `./meta_generator.sh` in terminal.
  4. Run `./run_generators.sh` in terminal (pass the desired values for the prediction horizon as parameters).
   * Ex: `./run_generators.sh 2 6 10`
  5. Run `./run_trials.sh` in `stn_generator_test` to start all trials.
  6. Results are saved in `aftcode2_0/rosbags`. (with timestamp appended to the end of the name)
   
**The naming scheme functions as follows:**

Scheme: `stN_pY_hX_bXR_XV_XL`
Ex: `st1_p120_hX_bBR_LV_DL` where solver_trial(test)=1, the players starting position is 120 (p), the solver prediction horizon is defined by the generator script, the ball (b) starts in the Bottom Right (BR) with a Low Velocity (LV) and moves in a Diagonal Line (DL).

	p is in range 0-120
	b can have variables; 1) Top Right (TR), Center Right (CR), Bottom Right (BR), 2) Low Velocity (LV), Medium Velocity (MV), High Velocity (HV), and 3) Horizontal Line (HL), Diagonal Line (DL), Vertical Line (VL)
	
	the system time is appended to the end of the naming scheme in `Y_M_D_h_m_s` format after the trial is run

**Note:** Run `aftcode2_0/scripts/simulators/ball_sim_test.py -h` in terminal to see test descriptions.
