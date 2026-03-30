//
// Created by admin on 2024/11/28.
//

#ifndef ADRC_CONTROL_IMP_INS_CAL_H
#define ADRC_CONTROL_IMP_INS_CAL_H

extern "C" {
void ImplementImuInstallationCalibration(   // 50ms
        double auto_drive,
        double calibration_cmd_imp,
        double tractor_speed,
        double e,
        double n,
        double u_imp,  // m
        double u_tractor,  // m
        double xtrack,
        double xhead,
        double line_a,
        double line_b,
        double line_c,
        double roll,
        double pitch,
        double yaw,
        double arm_imp_x,
        double arm_imp_y,
        double arm_imp_z,  // m
        double arm_tractor_z,  // m
        short *out_PitchOffsetIm,  // 0.01degree
        short *out_RollOffsetIm,
        long long int *out_HardwareStatus,
        unsigned short *out_CalibratePhaseIm,
        short *out_ArmXIm);
}

#endif //ADRC_CONTROL_IMP_INS_CAL_H
