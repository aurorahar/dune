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
      double path_angle;
      bool active;
      int n_vertices;
      int mode;
      uint16_t source;
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

      //! Vehicle state.
      bool m_estimate_received;
      bool m_in_maneuver;


      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Periodic(name, ctx),
        m_in_maneuver(false),
        m_estimate_received(false)
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

        param("Vertex Placements", m_args.vertices)
        .description("Position of the polygon vertices with respect to the center.")
        .defaultValue("0.0, 0.0");

        param("Number of Vertices", m_args.n_vertices)
        .description("Number of vertices in the obstacle polygon.")
        .defaultValue("1");

        param("Mode", m_args.mode)
        .description("Sets the behaviour of the obstacle. ")
        .defaultValue("0");

        param("Turn At Time", m_args.turn_time)
        .description("Time after simulation start the obstacle should take a turn.")
        .defaultValue("20.0")
        .units(Units::Second);

        param("Activated", m_args.active)
        .description("Activates the obstacle simulation.")
        .defaultValue("false");

        param("Source id", m_args.source)
        .description("Source id for sending the obstacle coordinates to Neptus.")
        .defaultValue("0x001F");

        param("Path angle", m_args.path_angle)
        .defaultValue("0.0")
        .units(Units::Degree);

        bind<IMC::EstimatedState>(this);
        bind<IMC::VehicleState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (!m_args.active){
          m_in_maneuver = false;
          m_estimate_received = false;
        }
        //! Set time step according to the execution frequency.
        c_ts = 1.0/getFrequency();
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

      void
      consume (const IMC::VehicleState* msg){
        if ( m_args.active ) {
          if (msg->op_mode == IMC::VehicleState::VS_MANEUVER && msg->maneuver_type != 461){
              if (!m_in_maneuver)
                m_maneuver_start = Clock::get();

              m_in_maneuver = true;
          }
          else{
            m_in_maneuver = false;
            m_estimate_received = false;
          }
        }
      }

      void onInitializeState(double lat, double lon){

        //! Initialize obstacle states.
        m_os.lat = lat;
        m_os.lon = lon;
        m_nepos.x = m_args.offset[0]*std::cos(Angles::radians(m_args.path_angle)) - m_args.offset[1]*std::sin(Angles::radians(m_args.path_angle));
        m_nepos.y = m_args.offset[0]*std::sin(Angles::radians(m_args.path_angle)) + m_args.offset[1]*std::cos(Angles::radians(m_args.path_angle));
        WGS84::displace(m_nepos.x, m_nepos.y,  &m_os.lat, &m_os.lon);
        m_os.cog = Math::Angles::radians(m_args.heading+m_args.path_angle);
        m_os.sog = m_args.speed;
        m_os.label = "ObstacleState";

        //! Message for Neptus
        m_es.lat = lat;
        m_es.lon = lon;
        m_es.x = m_nepos.x;
        m_es.y = m_nepos.y;
        m_es.psi = m_os.cog;
        m_es.u = m_os.sog;

        m_es.setSource(m_args.source);

        //! Initialize polygon.
        m_polygon.id = "ObstaclePolygon";
        m_polygon.feature_type = IMC::MapFeature::FTYPE_CONTOUREDPOLY;

        double dx, dy;

        for (int i = 0; i < m_args.n_vertices; i++)
        {
          IMC::MapPoint map_point;
          map_point.lat = m_os.lat;
          map_point.lon = m_os.lon;

          dx = m_args.vertices[2*i]*std::cos(Angles::radians(m_args.vertices[2*i+1])+Angles::radians(m_args.heading));
          dy = m_args.vertices[2*i]*std::sin(Angles::radians(m_args.vertices[2*i+1])+Angles::radians(m_args.heading));
          WGS84::displace(dx, dy,  &map_point.lat, &map_point.lon);

          m_polygon.feature.push_back(map_point);
        }
      }

      void
      consume(const IMC::EstimatedState * msg){
        if ( m_args.active ) {

          if (m_in_maneuver && !m_estimate_received){
            m_estimate_received = true;

            //! Initialize the obstacle position based on the lat/lon coordinates of the vehicle.
            double lat = msg->lat;
            double lon = msg->lon;

            WGS84::displace(msg->x, msg->y,  &lat, &lon);

            onInitializeState(lat,lon);
          }
        }
      }

      //! Sends the state of the obstacle.

      void
      sendState(void){

        //! Unicycle kinematics.
        double n = m_os.sog*std::cos(m_os.cog)*c_ts;
        double e = m_os.sog*std::sin(m_os.cog)*c_ts;
        m_nepos.x += n;
        m_nepos.y += e;

        //! Update longitude and latitude.
        WGS84::displace(n, e,  &m_os.lat, &m_os.lon);

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
              psid = Angles::radians(m_args.heading + 90.0);
            }
            break;
          case 2:
            //! Left turn.
            if (Clock::get()-m_maneuver_start > m_args.turn_time){
                psid = Angles::radians(m_args.heading -90.0);
            }
            break;
          case 3:
            //! Half right turn.
            if (Clock::get()-m_maneuver_start > m_args.turn_time){
                psid = Angles::radians(m_args.heading + 45.0);
            }
            break;
          case 4:
           //! Half left turn.
            if (Clock::get()-m_maneuver_start > m_args.turn_time){
                psid = Angles::radians(m_args.heading - 45.0);
            }
            break;
        }

        //! Compute heading rate.
        double r = Angles::minSignedAngle(m_os.cog, psid);
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
          p->lat =m_os.lat;
          p->lon =m_os.lon;

          double dx = m_args.vertices[2*i]*std::cos(Angles::radians(m_args.vertices[2*i+1])+m_os.cog);
          double dy = m_args.vertices[2*i]*std::sin(Angles::radians(m_args.vertices[2*i+1])+m_os.cog);

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
        m_es.r = m_r;
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
