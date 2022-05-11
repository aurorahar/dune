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
  //! @author Aurorahar
  namespace SendCommand
  {
    using DUNE_NAMESPACES;

    struct Arguments{
      std::vector<double> position;
      std::vector<double> offset;
    };


    struct Task: public DUNE::Tasks::Periodic
    {
       //! Arguments.
      Arguments m_args;

      //! Maneuver to be requested.
      IMC::Goto m_goto;
      InlineMessage<Maneuver> m_man;

      //! Command to be sent to the vehicle.
      IMC::VehicleCommand m_command;
      static const uint16_t c_id = 100;

      bool m_req_sent;
      bool m_calib_done;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_req_sent(false),
        m_calib_done(false)
      {
        param("Position", m_args.position)
        .description("Position of the reference origin. ")
        .size(2)
        .units(Units::Degree);

        param("Offset", m_args.offset)
        .description("Offset relative to the origin.")
        .size(2)
        .units(Units::Meter);


        bind<IMC::VehicleCommand>(this);
        bind<IMC::VehicleState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Set origin of the maneuver
        m_goto.lat = Math::Angles::radians(m_args.position[0]);
        m_goto.lon = Math::Angles::radians(m_args.position[1]);
        WGS84::displace(m_args.offset[0], m_args.offset[1],  &m_goto.lat, &m_goto.lon);
        m_goto.z = 0.0;
        m_goto.z_units = IMC::Z_ALTITUDE;
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

      void consume(const IMC::VehicleState * vs){
        if (vs->op_mode == IMC::VehicleState::VS_SERVICE)
          m_calib_done = true;
      }

      //! Show reply from supervisor.
      void consume(const IMC::VehicleCommand* reply_msg){
        if (!(reply_msg->request_id == m_command.request_id) || !(reply_msg->command == m_command.command)){
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


      void
      task(void)
      {
        //! Request a maneuver after the vehicle has entered service mode.

        if (m_calib_done && !m_req_sent){
          dispatch(m_command);
          inf("Requested maneuver 'Goto (%f, %f)'. ", Math::Angles::degrees(m_goto.lat), Math::Angles::degrees(m_goto.lon));
          m_req_sent = true;
        }

      }
    };
  }
}

DUNE_TASK
