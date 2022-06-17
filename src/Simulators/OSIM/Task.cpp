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
      std::vector<double> vertices;
      double heading;
      double speed;
      double r_max;
      double a_max;
      double u_max;
      double turn_time;
      bool save_data;
      int n_vertices;
      int mode;
    };

    struct Point{
      double x;
      double y;
    };

    struct Task: public DUNE::Tasks::Periodic
    {
       //! Arguments.
      Arguments m_args;

      //! Obstacle state and polygon to be sent.
      IMC::Target m_os;
      IMC::MapFeature m_polygon;
      IMC::EstimatedState m_es;

      //! Obstacle North East position and heading rate.
      Point m_nepos;
      double m_r;
      double m_maneuver_start;

      //! Step size of numerical integration.
      double c_ts;

      //! File for saving the data.
      std::ofstream m_data_file;
      bool m_in_maneuver;
      bool m_timer_on;
      double m_stop_time;
      const double c_time_thresh = 5.0;

      //! Vehicle state.
      bool m_estimate_received;
      Point m_vnepos;
      double m_ref_position[2];

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_in_maneuver(false),
        m_estimate_received(false),
        m_timer_on(false)
      {
        param("Offset", m_args.offset)
        .description("Offset of the obstacle position relative to vehicle start position.")
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

        param("Vertex Placements", m_args.vertices)
        .description("Position of the polygon vertices with respect to the center.");

        param("Number of Vertices", m_args.n_vertices)
        .defaultValue("1");

        param("Mode", m_args.mode)
        .defaultValue("0");

        param("Turn At Time", m_args.turn_time)
        .defaultValue("20.0")
        .units(Units::Second);

        bind<IMC::EstimatedState>(this);
        bind<IMC::VehicleState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        //! Set time step according to the execution frequency.
        c_ts = 1.0/getFrequency();
        m_stop_time = Clock::get();
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
        if (m_data_file.is_open()){
          inf("Closing file.");
          m_data_file.close();
        }
      }

      void
      consume (const IMC::VehicleState* msg){

        if (msg->op_mode == IMC::VehicleState::VS_MANEUVER){

          //! Check if the maneuvering flag is set and if we need to start saving the data.
          if (!m_in_maneuver && m_args.save_data){

            //! Get date and time for naming the file.
            std::string timestr = Format::getTimeDate();

            //! Remove illegal characters, year and seconds.
            std::string name = timestr.substr(5,2)+timestr.substr(8,2)+timestr.substr(11,2)+timestr.substr(14,2);

            //! Open the file in the designated MATLAB folder.
            m_data_file.open("../../MATLAB/DUNE_Experiments/log/"+ name +".txt");

            //! Confirm that we made the file.
            if (!m_data_file)
              inf("Cannot open file.");
            else{
              inf("Generated file %s to write.", name.c_str());
            }
            m_maneuver_start = Clock::get();
          }
          m_in_maneuver = true;
        }else{

          if (m_in_maneuver && !m_timer_on){
            m_stop_time = Clock::get();
            m_timer_on = true;
          }
          //! Wait a few seconds before closing the file.
          if (m_timer_on && Clock::get()- m_stop_time > c_time_thresh){
            if (m_data_file.is_open()){
              inf("Closing file.");
              m_data_file.close();
              }

            m_in_maneuver = false;
            m_timer_on = false;
            m_estimate_received = false;
          }
        }
      }

      void onInitializeObstacle(double lat, double lon){
        m_ref_position[0] = lat;
        m_ref_position[1] = lon;

        //! Initialize obstacle states.
        m_os.lat = lat;
        m_os.lon = lon;
        m_nepos.x = m_args.offset[0];
        m_nepos.y = m_args.offset[1];
        WGS84::displace(m_nepos.x, m_nepos.y,  &m_os.lat, &m_os.lon);
        m_os.cog = Math::Angles::radians(m_args.heading);
        m_os.sog = m_args.speed;
        m_os.label = "ObstacleState";

        //! Message for Neptus
        m_es.lat = lat;
        m_es.lon = lon;
        m_es.x = m_nepos.x;
        m_es.y = m_nepos.y;
        m_es.psi = m_os.cog;
        m_es.u = m_os.sog;

        m_es.setSource(0x001F); //xp2

        //! Initialize polygon.
        m_polygon.id = "ObstaclePolygon";
        m_polygon.feature_type = IMC::MapFeature::FTYPE_CONTOUREDPOLY;

        double dx, dy;

        for (int i = 0; i < m_args.n_vertices; i++)
        {
          IMC::MapPoint map_point;
          map_point.lat = m_os.lat;
          map_point.lon = m_os.lon;

          dx = m_nepos.x + m_args.vertices[2*i]*std::cos(Angles::radians(m_args.vertices[2*i+1])+Angles::radians(m_args.heading));
          dy = m_nepos.y + m_args.vertices[2*i]*std::sin(Angles::radians(m_args.vertices[2*i+1])+Angles::radians(m_args.heading));
          WGS84::displace(dx, dy,  &map_point.lat, &map_point.lon);

          m_polygon.feature.push_back(map_point);
        }
      }

      void
      consume(const IMC::EstimatedState * msg){

        if (m_in_maneuver && m_data_file.is_open()){

          //! Write data to file if we are in a maneuver.
          //! We save both the obstacle and vehicle state.
          if (m_in_maneuver && !m_estimate_received){
            m_estimate_received = true;

            //! Initialize the obstacle position based on the lat/lon coordinates of the vehicle.
            double lat = msg->lat;
            double lon = msg->lon;
            WGS84::displace(msg->x, msg->y,  &lat, &lon);

            onInitializeObstacle(lat,lon);
          }

          m_data_file << m_nepos.x<<"\n"<<m_nepos.y<<"\n"<<m_os.cog << "\n"
          << m_os.sog << "\n" << msg->x << "\n" << msg->y << "\n" << msg->psi
          << "\n" << msg->u << "\n" << msg->v << "\n" << m_r << "\n"<< msg->r << "\n";
        }


        //! Vehicle position for target tracking.
        m_vnepos.x = msg->x;
        m_vnepos.y = msg->y;
      }

      //! Sends the state of the obstacle.

      void
      sendState(void){

        //! Unicycle kinematics.
        double n = m_os.sog*std::cos(m_os.cog)*c_ts;
        double e = m_os.sog*std::sin(m_os.cog)*c_ts;
        m_nepos.x += n;
        m_nepos.y += e;

        //! Heading rate and acceleration of the obstacle.
        //! The mode determines what the obstacle should do.

        double a = m_args.a_max;
        double psid = m_os.cog;

        switch(m_args.mode){
          case 0:
            //! Constant heading.
            break;
          case 1:
            //! Right turn.
            if (Clock::get()-m_maneuver_start > m_args.turn_time){
              psid = Angles::radians(m_args.heading+90.0);
            }
            break;
          case 2:
            //! Left turn.
            if (Clock::get()-m_maneuver_start > m_args.turn_time){
                psid = Angles::radians(m_args.heading -90.0);
            }
            break;
          case 3:
            //! Pure pursuit tracking.
            psid = Coordinates::getBearing(m_nepos, m_vnepos);
            break;
        }

        //! Compute heading rate.
        double r = Angles::minSignedAngle(m_os.cog, psid);

        //! Update longitude and latitude.
        WGS84::displace(n, e,  &m_os.lat, &m_os.lon);
        m_r = std::max(std::min(r,m_args.r_max), -m_args.r_max);
        m_os.cog += m_r*c_ts;
        double u = m_os.sog+std::max(std::min(a,m_args.a_max), -m_args.a_max)*c_ts;

        //! Ensure that speed is between 0 and max and course between -pi and pi
        m_os.sog = std::max(std::min(u, m_args.u_max), 0.0);
        m_os.cog = Angles::normalizeRadian(m_os.cog);

        //! Update vertices according to center position.
        int i = 0;

        for (IMC::MapPoint * p : m_polygon.feature)
        {
          p->lat =m_ref_position[0];
          p->lon =m_ref_position[1];

          double dx = m_nepos.x + m_args.vertices[2*i]*std::cos(Angles::radians(m_args.vertices[2*i+1])+m_os.cog);
          double dy = m_nepos.y + m_args.vertices[2*i]*std::sin(Angles::radians(m_args.vertices[2*i+1])+m_os.cog);

          WGS84::displace(dx, dy,  &p->lat, &p->lon);
          i++;
        }

        //! Send the messages
        dispatch(m_polygon);
        dispatch(m_os);

        //! Message for Neptus
        m_es.x = m_nepos.x;
        m_es.y = m_nepos.y;
        m_es.psi = m_os.cog;
        m_es.u = m_os.sog;
        dispatch(m_es);
      }

      void
      task(void)
      {
        //! Send the state of the obstacle regularly, but only if the vehicle
        //! is in a maneuver and we have received the lat/lon coordinates.
        if (m_in_maneuver && m_estimate_received){
           sendState();
        }
      }
    };
  }
}

DUNE_TASK
