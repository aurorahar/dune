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
      Arguments args;

      //! Obstacle state to be sent.
      IMC::Target os;

      //! Maneuver to be requested.
      IMC::Goto man_goto;
      InlineMessage<Maneuver> m_man;

      //! Command to be sent to the vehicle.
      IMC::VehicleCommand m_command;
      static const uint16_t ID = 100;

      //! Time interval for numerical integration.
      double ts;

      //
      double c_ini_time;
      const float c_init_timeout = 15.0;
      double m_req_sent;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_req_sent(false)
      {
        param("Position", args.position)
        .description("Position of the reference origin. ")
        .size(2)
        .units(Units::Degree);

        param("Offset", args.offset)
        .description("Offset of the obstacle position relative to the origin.")
        .size(2)
        .units(Units::Meter);

        param("Heading", args.heading)
        .description("Initial orientation of the obstacle.")
        .defaultValue("0.0")
        .units(Units::Degree);

        param("Speed", args.speed)
        .description("Initial speed of the obstacle.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

         param("Maximum Heading Rate", args.r_max)
        .description("Maximum obstacle heading rate.")
        .defaultValue("0.0")
        .units(Units::RadianPerSecond);

        param("Maximum Speed", args.u_max)
        .description("Maximum obstacle speed.")
        .defaultValue("0.0")
        .units(Units::MeterPerSecond);

        param("Maximum Acceleration", args.a_max)
        .description("Maximum obstacle acceleration.")
        .defaultValue("0.0")
        .units(Units::MeterPerSquareSecond);

        bind<IMC::VehicleCommand>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Initialize times.
        c_ini_time = Clock::get();

        //! Set time step according to the execution frequency.
        ts = 1.0/getFrequency();

        //! Obstacle state.
        os.lat = Math::Angles::radians(args.position[0]);
        os.lon = Math::Angles::radians(args.position[1]);
        WGS84::displace(args.offset[0], args.offset[1],  &os.lat, &os.lon);
        os.cog = Math::Angles::radians(args.heading);
        os.sog = args.speed;
        os.label = "ObstacleState";

        //! Set origin of the maneuver
        man_goto.lat = Math::Angles::radians(args.position[0]);
        man_goto.lon = Math::Angles::radians(args.position[1]);
        man_goto.z = 0.0;
        man_goto.z_units = IMC::Z_ALTITUDE;

        //! Set the desired position in North and East directions.
        //! This is chosen arbitrary just to test the path controller.

        WGS84::displace(-50, 30,  &man_goto.lat, &man_goto.lon);
        man_goto.speed = 1.5;
        man_goto.speed_units = IMC::SUNITS_METERS_PS;

        //! Inline message.
        m_man.set(&man_goto);

        //! Set the command attributes, these are necessary for the command to be accepted.
        //! We request execution of a maneuver.
        m_command.maneuver = m_man;
        m_command.type = IMC::VehicleCommand::VC_REQUEST;
        m_command.command = IMC::VehicleCommand::VC_EXEC_MANEUVER;
        m_command.request_id = ID;

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
        double r = 0.0;
        double a = 0.0;
        //! Unicycle kinematics.
        double n = os.sog*std::cos(os.cog)*ts;
        double e = os.sog*std::sin(os.cog)*ts;

        //! Update longitudal and latidual coordinates.
        WGS84::displace(n, e,  &os.lat, &os.lon);

        os.cog += std::max(std::min(r,args.r_max), -args.r_max)*ts;
        double u = os.sog+std::max(std::min(a,args.a_max), -args.a_max)*ts;
        //! Ensure that speed is between 0 and max.
        os.sog = std::max(std::min(u, args.u_max), 0.0);

        dispatch(os);
      }

      //! Proportional heading controller.

      double headingController(const double desiredHeading, const double gain){
        return gain*Angles::minSignedAngle(os.cog, desiredHeading);
      }

      //! Proportional speed controller.

      double speedController(const double desiredSpeed, const double gain){
        return  -gain*(os.sog-desiredSpeed);
      }

      bool hasInit(void){
        double c_time = Clock::get();
        return c_time - c_ini_time > c_init_timeout;
      }

      void
      task(void)
      {
        //! Request a maneuver after the vehicle has initialized. Now we wait 15
        //! seconds to send, but could we wait for a control state update. To be
        //! implemented

        if (hasInit() && !m_req_sent){
          dispatch(m_command);
          inf("Requested maneuver 'Goto (%f, %f)'. ", Math::Angles::degrees(man_goto.lat), Math::Angles::degrees(man_goto.lon));
          m_req_sent = true;
        }

        //! Send obstacle state regularly
        sendState();
      }
    };
  }
}

DUNE_TASK
