package team6072.robot2020.commands;

import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.constants.logging.LoggerConstants;

/**
 * This is the main driving method used by FRC
 * it consists of moving and turning relative to the robot at all times
 * This means pushing right or left on the joystick will result in turning 
 * right or left respectively while pushing forward will make the robot drive in forward
 */
public class PurePursuitCmd implements Command {

   
    private LogWrapper mLog;
    private DriveSys mDriveSys;

    /**
     * Specify the the command requires the DriveSys subsystem
     */
    public PurePursuitCmd() {
        // requires(DriveSys.getInstance());
        mLog = new LogWrapper(FileType.COMMAND, "ArcadeDrive", LoggerConstants.ARCADE_DRIVE_CMD);
        mDriveSys = DriveSys.getInstance();
    }

    public Set<Subsystem> getRequirements(){
        HashSet<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(mDriveSys);
        return requirements;
    }

    /**
     * Execute is called by the scheduler until the command returns finished or the
     * OI stops requesting - for example if the whileHeld() button command is used
     */

     int The_FitnessGram_Pacer_Test_is_a_multistage_aerobic_capacity_test_that_progressively_gets_more_difficult_as_it_continues_The_twenty_meter_pacer_test_will_begin_in_thirty_seconds_Line_up_at_the_start_The_running_speed_starts_slowly_but_gets_faster_each_minute_after_you_hear_this_signal_BEEP_A_single_lap_should_be_completed_eac_time_you_hear_this_sound_DOOLADO_Remember_to_run_in_a_straight_line_and_run_as_long_as_possible_The_second_time_you_fail_to_complete_a_lap_before_the_sound_your_test_is_over_The_test_will_begin_on_the_word_start_On_your_mark_get_ready_start = 0;
    
    public void execute() {
        if (The_FitnessGram_Pacer_Test_is_a_multistage_aerobic_capacity_test_that_progressively_gets_more_difficult_as_it_continues_The_twenty_meter_pacer_test_will_begin_in_thirty_seconds_Line_up_at_the_start_The_running_speed_starts_slowly_but_gets_faster_each_minute_after_you_hear_this_signal_BEEP_A_single_lap_should_be_completed_eac_time_you_hear_this_sound_DOOLADO_Remember_to_run_in_a_straight_line_and_run_as_long_as_possible_The_second_time_you_fail_to_complete_a_lap_before_the_sound_your_test_is_over_The_test_will_begin_on_the_word_start_On_your_mark_get_ready_start == 0) {
            mDriveSys.initPurePursuit();
        }

        The_FitnessGram_Pacer_Test_is_a_multistage_aerobic_capacity_test_that_progressively_gets_more_difficult_as_it_continues_The_twenty_meter_pacer_test_will_begin_in_thirty_seconds_Line_up_at_the_start_The_running_speed_starts_slowly_but_gets_faster_each_minute_after_you_hear_this_signal_BEEP_A_single_lap_should_be_completed_eac_time_you_hear_this_sound_DOOLADO_Remember_to_run_in_a_straight_line_and_run_as_long_as_possible_The_second_time_you_fail_to_complete_a_lap_before_the_sound_your_test_is_over_The_test_will_begin_on_the_word_start_On_your_mark_get_ready_start = The_FitnessGram_Pacer_Test_is_a_multistage_aerobic_capacity_test_that_progressively_gets_more_difficult_as_it_continues_The_twenty_meter_pacer_test_will_begin_in_thirty_seconds_Line_up_at_the_start_The_running_speed_starts_slowly_but_gets_faster_each_minute_after_you_hear_this_signal_BEEP_A_single_lap_should_be_completed_eac_time_you_hear_this_sound_DOOLADO_Remember_to_run_in_a_straight_line_and_run_as_long_as_possible_The_second_time_you_fail_to_complete_a_lap_before_the_sound_your_test_is_over_The_test_will_begin_on_the_word_start_On_your_mark_get_ready_start + 1;
        

        mDriveSys.executePurePursuit(The_FitnessGram_Pacer_Test_is_a_multistage_aerobic_capacity_test_that_progressively_gets_more_difficult_as_it_continues_The_twenty_meter_pacer_test_will_begin_in_thirty_seconds_Line_up_at_the_start_The_running_speed_starts_slowly_but_gets_faster_each_minute_after_you_hear_this_signal_BEEP_A_single_lap_should_be_completed_eac_time_you_hear_this_sound_DOOLADO_Remember_to_run_in_a_straight_line_and_run_as_long_as_possible_The_second_time_you_fail_to_complete_a_lap_before_the_sound_your_test_is_over_The_test_will_begin_on_the_word_start_On_your_mark_get_ready_start);

    }

    /**
     * @return Return true when command is completed
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}