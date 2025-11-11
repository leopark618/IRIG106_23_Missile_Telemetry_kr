#include "ground_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 지상국 제어 명령 처리 구현
 * ============================================================ */

bool GroundControl_ReceiveCommand(uint8_t *data, uint32_t len,
                                   GroundControlCommand *cmd)
{
    if (!data || !cmd || len < sizeof(GroundControlCommand)) {
        return false;
    }
    
    memcpy(cmd, data, sizeof(GroundControlCommand));
    return true;
}

bool GroundControl_ValidateCommand(GroundControlCommand *cmd)
{
    if (!cmd) return false;
    
    if (cmd->header != IRIGFIX_COMMAND_HEADER) {
        return false;
    }
    
    if (!GroundControl_ValidateCRC(cmd)) {
        return false;
    }
    
    return true;
}

bool GroundControl_ValidateCRC(GroundControlCommand *cmd)
{
    if (!cmd) return false;
    
    uint16_t calculated_crc = 0;
    uint8_t *data = (uint8_t *)cmd;
    for (size_t i = 0; i < sizeof(GroundControlCommand) - 2; i++) {
        calculated_crc ^= data[i];
    }
    
    return calculated_crc == cmd->checksum;
}

bool GroundControl_ProcessCommand(GroundControlCommand *cmd,
                                   ControlState *state)
{
    if (!cmd || !state) return false;
    
    if (!GroundControl_ValidateCommand(cmd)) {
        return false;
    }
    
    switch (cmd->cmd_type) {
        case CMD_THRUST_UPDATE:
            if (cmd->payload.thrust.thrust_percent >= PT_THRUST_MIN &&
                cmd->payload.thrust.thrust_percent <= PT_THRUST_MAX) {
                state->current_thrust = cmd->payload.thrust.thrust_percent;
                GroundControl_SetThrust(state->current_thrust);
            }
            break;
            
        case CMD_RUDDER_UPDATE:
            if (cmd->payload.rudder.rudder_angle >= -PT_RUDDER_MAX_ANGLE &&
                cmd->payload.rudder.rudder_angle <= PT_RUDDER_MAX_ANGLE) {
                state->current_rudder_angle = cmd->payload.rudder.rudder_angle;
                GroundControl_SetRudder(state->current_rudder_angle);
            }
            break;
            
        case CMD_ELEVON_UPDATE:
            if (cmd->payload.elevon.elevon_angle >= -PT_ELEVON_MAX_ANGLE &&
                cmd->payload.elevon.elevon_angle <= PT_ELEVON_MAX_ANGLE) {
                state->current_elevon_angle = cmd->payload.elevon.elevon_angle;
                GroundControl_SetElevon(state->current_elevon_angle);
            }
            break;
            
        case CMD_TRAJECTORY_CHANGE:
            state->target_heading = cmd->payload.trajectory.target_heading;
            state->target_pitch = cmd->payload.trajectory.target_pitch;
            state->target_roll = cmd->payload.trajectory.target_roll;
            break;
            
        case CMD_SPEED_INCREASE:
            if (state->current_thrust < PT_THRUST_MAX) {
                state->current_thrust += cmd->payload.speed.speed_change;
                if (state->current_thrust > PT_THRUST_MAX) {
                    state->current_thrust = PT_THRUST_MAX;
                }
                GroundControl_SetThrust(state->current_thrust);
            }
            break;
            
        case CMD_SPEED_DECREASE:
            if (state->current_thrust > PT_THRUST_MIN) {
                state->current_thrust -= cmd->payload.speed.speed_change;
                if (state->current_thrust < PT_THRUST_MIN) {
                    state->current_thrust = PT_THRUST_MIN;
                }
                GroundControl_SetThrust(state->current_thrust);
            }
            break;
            
        default:
            return false;
    }
    
    state->last_command_time = cmd->timestamp;
    state->is_command_valid = true;
    
    return true;
}

void GroundControl_UpdateState(ControlState *state, GroundControlCommand *cmd)
{
    if (!state || !cmd) return;
    GroundControl_ProcessCommand(cmd, state);
}

void GroundControl_ApplyControls(ControlState *state)
{
    if (!state) return;
    
    GroundControl_SetThrust(state->current_thrust);
    GroundControl_SetRudder(state->current_rudder_angle);
    GroundControl_SetElevon(state->current_elevon_angle);
}

void GroundControl_SetThrust(float percent)
{
    if (percent < PT_THRUST_MIN) percent = PT_THRUST_MIN;
    if (percent > PT_THRUST_MAX) percent = PT_THRUST_MAX;
}

void GroundControl_SetRudder(float angle)
{
    if (angle < -PT_RUDDER_MAX_ANGLE) angle = -PT_RUDDER_MAX_ANGLE;
    if (angle > PT_RUDDER_MAX_ANGLE) angle = PT_RUDDER_MAX_ANGLE;
}

void GroundControl_SetElevon(float angle)
{
    if (angle < -PT_ELEVON_MAX_ANGLE) angle = -PT_ELEVON_MAX_ANGLE;
    if (angle > PT_ELEVON_MAX_ANGLE) angle = PT_ELEVON_MAX_ANGLE;
}

void GroundControl_CheckTimeout(ControlState *state)
{
    if (!state) return;
    
    uint64_t current_time = 0;
    
    if ((current_time - state->last_command_time) > PT_COMMAND_TIMEOUT_MS) {
        state->is_command_valid = false;
        state->current_thrust = PT_THRUST_MIN;
        GroundControl_SetThrust(PT_THRUST_MIN);
    }
}
