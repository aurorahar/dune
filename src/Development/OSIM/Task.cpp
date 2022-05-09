//***************************************************************************
// Copyright 2007-2022 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Aurorahar                                                        *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Development
{
  //! Obstacle simulator.
  //!
  //! Task to simulate a moving obstacle and transmit the position and
  //! velocity to the vehicle. Collision avoidance implemented in the path
  //! controller.
  //!
  //! ***Currently testing sending maneuvers to the vehicle from this task**
  //! @author Aurorahar
  namespace OSIM
  {
    using DUNE_NAMESPACES;

    struct Arguments{
      std::vector<double> position;
      std::vector<double> offset;
      double heading;
      double speed;
      double r_max;
      double a_max;
      double u_max;
    };


    struct Task: public DUNE::Tasks::Periodic
    {
       //! Arguments.
      Arguments m_args;

      //! Obstacle state to be sent.
      IMC::Target m_os;

      //! Maneuver to be requested.
      IMC::Goto m_goto;
      InlineMessage<Maneuver> m_man;

      //! Command to be sent to the vehicle.
      IMC::VehicleCommand m_command;
      static const uint16_t c_id = 100;

      //! Time interval for numerical integration.
      double c_ts;

      //
      bool m_req_sent;
      bool m_calib_done;
      bool m_speed_ok;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_req_sent(false),
        m_calib_done(false),
        m_speed_ok(false)
      {
        param("Position", m_args.position)
        .description("Position of the reference origin. ")
        .size(2)
        .units(Units::Degree);

        param("Offset", m_args.offset)
        .description("Offset of the obstacle position relative to the origin.")
        .size(2)
        .units(Units::Meter);

        param("Heading", m_args.heading)
        .description("Initial orientation of the obstacle.")
        .defaultValue("0.0")
        .units(Units::Degree);

        param("Speed", m_args.speed)
        .description("Initial speed of the obstacle.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

         param("Maximum Heading Rate", m_args.r_max)
        .description("Maximum obstacle heading rate.")
        .defaultValue("0.0")
        .units(Units::RadianPerSecond);

        param("Maximum Speed", m_args.u_max)
        .description("Maximum obstacle speed.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

        param("Maximum Acceleration", m_args.a_max)
        .description("Maximum obstacle acceleration.")
        .defaultValue("0.0")
        .units(Units::MeterPerSquareSecond);

        bind<IMC::VehicleCommand>(this);
        bind<IMC::VehicleState>(this);
        //bind<IMC::EstimatedState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Set time step according to the execution frequency.
        c_ts = 1.0/getFrequency();

        //! Obstacle state.
        m_os.lat = Math::Angles::radians(m_args.position[0]);
        m_os.lon = Math::Angles::radians(m_args.position[1]);
        WGS84::displace(m_args.offset[0], m_args.offset[1],  &m_os.lat, &m_os.lon);
        m_os.cog = Math::Angles::radians(m_args.heading);
        m_os.sog = m_args.speed;
        m_os.label = "ObstacleState";

        //! Set origin of the maneuver
        m_goto.lat = Math::Angles::radians(m_args.position[0]);
        m_goto.lon = Math::Angles::radians(m_args.position[1]);
        m_goto.z = 0.0;
        m_goto.z_units = IMC::Z_ALTITUDE;

        //! Set the desired position in North and East directions.
        //! This is chosen arbitrary just to test the path controller.

        WGS84::displace(-60.0, 40.0,  &m_goto.lat, &m_goto.lon);
        m_goto.speed = 1.5;
        m_goto.speed_units = IMC::SUNITS_METERS_PS;

        //! Inline message.
        m_man.set(&m_goto);

        //! Set the command attributes, these are necessary for the command to be accepted.
        //! We request execution of a maneuver.
        m_command.maneuver = m_man;
        m_command.type = IMC::VehicleCommand::VC_REQUEST;
        m_command.command = IMC::VehicleCommand::VC_EXEC_MANEUVER;
        m_command.request_id = c_id;

        //! Note that, if external control is enabled, maneuvers can be transmitted directly to the vehicle,
        //! and we do not have request it by sending a command as we do here. However, the maneuver will not
        //! be  stopped when the goal is reached.
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        inf("Starting: %s", resolveEntity(getEntityId()).c_str());
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

    //!  void consume(const IMC::EstimatedState * es){
    //!    m_speed_ok = es->u > m_args.u_max ? true : false;
    //!  }

      void consume(const IMC::VehicleState * vs){
        if (vs->op_mode == IMC::VehicleState::VS_SERVICE)
          m_calib_done = true;
      }

      //! Show reply from supervisor.
      void consume(const IMC::VehicleCommand* reply_msg){
        if (!(reply_msg->request_id == m_command.request_id) || !(reply_msg->command == m_command.command)){
          inf("Request ids and/or commands do not match. Returning. ");
          return;
        }

        if (reply_msg->type == IMC::VehicleCommand::VC_SUCCESS){
          inf("Received message: %s", reply_msg->info.c_str());
          return;
        }

        if (reply_msg->type == IMC::VehicleCommand::VC_FAILURE){
          inf("Received message: %s", reply_msg->info.c_str());
          return;
        }
      }

      //! Sends the state of the obstacle.

      void
      sendState(void){
        //! Speed and acceleration. To be updated.
        double r = 0.1;
        double a = 0.0;
        //! Unicycle kinematics.
        double n = m_os.sog*std::cos(m_os.cog)*c_ts;
        double e = m_os.sog*std::sin(m_os.cog)*c_ts;

        //! Update longitudal and latidual coordinates.
        WGS84::displace(n, e,  &m_os.lat, &m_os.lon);

        m_os.cog += std::max(std::min(r,m_args.r_max), -m_args.r_max)*c_ts;
        double u = m_os.sog+std::max(std::min(a,m_args.a_max), -m_args.a_max)*c_ts;
        //! Ensure that speed is between 0 and max.
        m_os.sog = std::max(std::min(u, m_args.u_max), 0.0);
        m_os.cog = fmod(m_os.cog, 2*c_pi);

        dispatch(m_os);
      }

      //! Proportional heading controller.

      double headingController(const double desiredHeading, const double gain){
        return gain*Angles::minSignedAngle(m_os.cog, desiredHeading);
      }

      //! Proportional speed controller.

      double speedController(const double desiredSpeed, const double gain){
        return  -gain*(m_os.sog-desiredSpeed);
      }

      void
      task(void)
      {
        //! Request a maneuver after the vehicle has initialized. We wait until
        //! the vehicle is in service mode. Then we send the obtacle state once
        //! the speed of the vehicle is higher than the obstacle maximum speed

        if (m_calib_done && !m_req_sent){
          dispatch(m_command);
          inf("Requested maneuver 'Goto (%f, %f)'. ", Math::Angles::degrees(m_goto.lat), Math::Angles::degrees(m_goto.lon));
          m_req_sent = true;
        }


        sendState();
      }
    };
  }
}

DUNE_TASK
