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

namespace Simulators
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
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
      bool save_data;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
       //! Arguments.
      Arguments m_args;
      //! Obstacle state to be sent.
      IMC::Target m_os;
      //! Save North East position.
      double m_nepos[2];
      //! Step size of numerical integration.
      double c_ts;
      //! File for saving the data.
      std::ofstream data_file;
      bool m_in_maneuver; //! We only want to save the data from when the vehicle is in a maneuver

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),m_in_maneuver(false)
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

        param("Save Data", m_args.save_data)
        .defaultValue("false");
        bind<IMC::EstimatedState>(this);
        bind<IMC::VehicleState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Set time step according to the execution frequency.
        c_ts = 1.0/getFrequency();

        //! Initialize obstacle states.
        m_os.lat = Math::Angles::radians(m_args.position[0]);
        m_os.lon = Math::Angles::radians(m_args.position[1]);
        WGS84::displace(m_args.offset[0], m_args.offset[1],  &m_os.lat, &m_os.lon);
        m_nepos[0] = m_args.offset[0];
        m_nepos[1] = m_args.offset[1];
        m_os.cog = Math::Angles::radians(m_args.heading);
        m_os.sog = m_args.speed;
        m_os.label = "ObstacleState";
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
        if (data_file.is_open()){
          inf("Closing file.");
          data_file.close();
        }
      }

      void
      consume (const IMC::VehicleState* msg){
        if (msg->op_mode == IMC::VehicleState::VS_MANEUVER){
          //! Check the maneuvering flag is already set or if we need to generate a new file.
          if (!m_in_maneuver && m_args.save_data){

            //! Get date and time.
            std::string timestr = Format::getTimeDate();

            //! Remove illegal characters, year and seconds.
            std::string name = timestr.substr(5,2)
            +timestr.substr(8,2)+ timestr.substr(11,2)+ timestr.substr(14,2);

            //! Make file in the MATLAB folder.
            data_file.open("../../MATLAB/log/"+ name +".txt");

            //! Check that we made the file..
            if (!data_file)
              inf("Cannot open file.");
            else{
              inf("Generated file %s to write.", name.c_str());
              m_in_maneuver = true;
            }
          }
        }else{
          //! Check if we need to close the file.
          if (m_in_maneuver){
            if (data_file.is_open()){
              inf("Closing file.");
              data_file.close();
            }
            m_in_maneuver = false;
          }
        }
      }

      void
      consume(const IMC::EstimatedState * msg){
        if (m_in_maneuver){
          //! Save data to file if we are in a maneuver.
          data_file << m_nepos[0]<<"\n"<<m_nepos[1]<<"\n"<<m_os.cog << "\n"
          << m_os.sog << "\n" << msg->x << "\n" << msg->y << "\n" << msg->psi
          << "\n" << msg->u << "\n" << msg->v << "\n";
        }
      }


      //! Sends the state of the obstacle.

      void
      sendState(void){

        //! Heading rate and acceleration of the obstacle. To be updated.
        //! Now making the obstacle move in a circle with constant speed.
        double r = 0.05;
        double a = 0.0;

        //! Unicycle kinematics.
        double n = m_os.sog*std::cos(m_os.cog)*c_ts;
        double e = m_os.sog*std::sin(m_os.cog)*c_ts;
        m_nepos[0] += n;
        m_nepos[1] += e;

        //! Update longitude and latitude
        WGS84::displace(n, e,  &m_os.lat, &m_os.lon);

        m_os.cog += std::max(std::min(r,m_args.r_max), -m_args.r_max)*c_ts;
        double u = m_os.sog+std::max(std::min(a,m_args.a_max), -m_args.a_max)*c_ts;

        //! Ensure that speed is between 0 and max and course between -pi and pi
        m_os.sog = std::max(std::min(u, m_args.u_max), 0.0);
        m_os.cog = Angles::normalizeRadian(m_os.cog);

        dispatch(m_os);
      }

      void
      task(void)
      {
        //! Send the state of the obstacle regularly.
        sendState();
      }
    };
  }
}

DUNE_TASK
