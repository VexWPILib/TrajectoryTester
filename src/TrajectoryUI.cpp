// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "TrajectoryUI.h"

namespace vpi {

  namespace ui {

    using namespace vpi;

    const char *btnmMap[] = {"1", "\n", "2", "\n", "3", ""};

    lv_obj_t *datBtnm;

    void btnmEventHandler(lv_obj_t * obj, lv_event_t e){
      if(e == LV_EVENT_VALUE_CHANGED) {
        //const char * txt = lv_btnmatrix_get_active_btn_text(obj);
        int i = lv_btnmatrix_get_active_btn(obj);
        printf("btnmEventHandler %d\n",i);
        TrajectoryConfig config(1_ftps, 1 * ftps2);

        VexGpsPose2d vstart(-4_ft, -4_ft, 45_deg);
        VexGpsPose2d vmid(0_ft, 0_ft, 45_deg);
        VexGpsPose2d vend(4_ft, -4_ft, 135_deg);

        Trajectory t;
        std::vector<Translation2d> wp = {{0_ft, 0_ft}};
        //std::vector<Translation2d> wp = {{2_ft, -2_ft}, {2_ft,2_ft}};
        //std::vector<Translation2d> wp = {{2_ft, -2_ft}, {0_ft, 0_ft}, {2_ft,2_ft}};
        t = TrajectoryGenerator::GenerateTrajectory(vstart, wp, vend, config);

        std::vector<Trajectory::State> states = t.States();
        printf("states.size() == %d\n", states.size());
        for(int k = 1; k < states.size(); ++k) {
          ChassisSpeeds cs;
          cs.vx = states[k].velocity;
          cs.omega = states[k].velocity * states[k].curvature;
          VexGpsPose2d curGpsPos = states[k].pose;
          printf("%d - At %.3f (%d, %d) hi=%d hg=%d a=%.3f vx=%.3f omega=%.3f\n", k,
                states[k].t.convert(second), 
                (int)states[k].pose.X().convert(inch),(int)states[k].pose.Y().convert(inch),
                (int)states[k].pose.Rotation().ToAngle().convert(degree),
                (int)curGpsPos.Theta().convert(degree),
                states[k].acceleration.convert(ftps2),
                cs.vx.convert(ftps), cs.omega.convert(radps));
          QTime timeToWait = states[k].t - states[k - 1].t;
          wait(timeToWait.convert(millisecond), msec);
        } 

        vex::motor ml1 = motor(ROBOT_LEFT_MOTOR_1, ROBOT_GEAR_SETTING, ROBOT_LEFT_MOTOR_1_REV);
        vex::motor ml2 = motor(ROBOT_LEFT_MOTOR_2, ROBOT_GEAR_SETTING, ROBOT_LEFT_MOTOR_2_REV);
        vex::motor ml3 = motor(ROBOT_LEFT_MOTOR_3, ROBOT_GEAR_SETTING, ROBOT_LEFT_MOTOR_3_REV);
        vex::motor mr1 = motor(ROBOT_RIGHT_MOTOR_1, ROBOT_GEAR_SETTING, ROBOT_RIGHT_MOTOR_1_REV);
        vex::motor mr2 = motor(ROBOT_RIGHT_MOTOR_2, ROBOT_GEAR_SETTING, ROBOT_RIGHT_MOTOR_2_REV);
        vex::motor mr3 = motor(ROBOT_RIGHT_MOTOR_3, ROBOT_GEAR_SETTING, ROBOT_RIGHT_MOTOR_3_REV);
        vex::motor_group mg_left{ml1, ml2 ,ml3};
        vex::motor_group mg_right{mr1, mr2, mr3};
        DifferentialDriveChassis m_chassis(mg_left, mg_right, 
                                              ROBOT_WHEEL_TRACK, ROBOT_WHEEL_DIAMETER, 
                                              ROBOT_GEAR_SETTING, ROBOT_GEAR_RATIO, vex::brakeType::coast);

        if(i == 0) {
          printf("0\n");
          SimpleWaypointFollower swf(m_chassis);
          swf.FollowTrajectory({vstart, vmid, vend}, 2_ftps);
        } else if (i == 1) {
          printf("1\n");
          SimpleTrajectoryFollower stf(m_chassis);
          stf.FollowTrajectory(t);
        } else if (i == 2) {
          printf("2\n");
          Pose2d tol({.5_in, .5_in}, 2 * degree);
          RamseteTrajectoryFollower rtf(m_chassis, tol);
          rtf.FollowTrajectory(t);
        }

        printf("Done\n");
      }
    }

    void ramseteInit(){
      // Set the full screen background to Black
      static lv_style_t style_screen;
      // LV_STYLE_BG_COLOR
      lv_style_set_bg_color(&style_screen, LV_STATE_DEFAULT, LV_COLOR_BLACK);
      lv_obj_add_style(lv_scr_act(),LV_OBJ_PART_MAIN, &style_screen);

      static lv_style_t style1;
      static lv_style_t style2;
      static lv_style_t style3;
      lv_style_set_bg_color(&style1, LV_STATE_DEFAULT, LV_COLOR_BLACK);
      lv_style_set_bg_color(&style1, LV_STATE_PRESSED, LV_COLOR_GRAY);
      lv_style_set_bg_color(&style2, LV_STATE_CHECKED | LV_STATE_FOCUSED | LV_STATE_DEFAULT, LV_COLOR_PURPLE);
      lv_style_set_text_color(&style3, LV_STATE_DEFAULT, LV_COLOR_WHITE);

      // Create the DAT button matrix
      datBtnm = lv_btnmatrix_create(lv_scr_act(), NULL);
      lv_btnmatrix_set_map(datBtnm, btnmMap);
      lv_obj_set_size(datBtnm, 100, 200);
      lv_obj_set_pos(datBtnm, 0, 0);
      lv_obj_align(datBtnm, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
      lv_obj_set_event_cb(datBtnm, btnmEventHandler);
      lv_obj_add_style(datBtnm,LV_BTNMATRIX_PART_BG, &style1);
      lv_obj_add_style(datBtnm,LV_BTNMATRIX_PART_BTN, &style2);

    }

  } // namespace ui
} // namespace vpi