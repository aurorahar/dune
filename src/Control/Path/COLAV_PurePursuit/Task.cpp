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

namespace Control
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Aurorahar
  //! Original pure pursuit controller by Eduardo Marques

  namespace Path
  {
    namespace COLAV_PurePursuit
    {
      using DUNE_NAMESPACES;

      struct Task: public DUNE::Control::PathController
      {
        IMC::DesiredHeading m_heading;


        //! Collision Avoidance
        double coll_cone[2];
        double dsep;
        bool m_in_colreg;
        bool m_in_ca;
        int m_turn_dir;
        //! Obstacle State
        double opos[2];
        double opsi;
        double ou;
        bool m_os_received;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_os_received(false),
          m_in_colreg(false),
          m_in_ca(false)
        {
          param("Separation Distance", dsep)
          .defaultValue("10.0")
          .units(Units::Meter);

          bind<IMC::Target>(this);
        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();
        }
        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }
        void
        onPathActivation(void)
        {
          enableControlLoops(IMC::CL_YAW);
        }
        void
        onPathDeactivation(void)
        {
          disableControlLoops(IMC::CL_YAW);
        }

        void consume(const IMC::Target * os){

          //! Save obstacle measurements
          opos[0] = os->lat;
          opos[1] = os->lon;
          opsi = os->cog;
          ou = os->sog;

          m_os_received = true;
        }

        void shouldCA(const IMC::EstimatedState& vs, double course){

          double d, alpha, beta;
          Coordinates::WGS84::getNEBearingAndRange(vs.lat, vs.lon, opos[0], opos[1], & alpha, & d);

          m_in_colreg = d < dsep;

          //! Should not happen but if we are inside the collision region
          //! this heading will bring the vehicle out of the reguib and we
          //! avoid error when calling asin.

          if( m_in_colreg ){
            debug("Distance to obstacle is %f [m] -- Inside collision region", d);
            beta = c_pi - std::asin(d/dsep);
          } else {
            beta = std::asin(dsep/d);
          }

          double u = vs.u;
          double v = vs.v;

          double speed = std::sqrt(u*u + v*v);

          if (ou > speed){
            // Cannot compute the velocity compensation term.
            signalError(DTR("Collision cone error"));
            return;
          }

          coll_cone[0] = alpha - beta + std::asin(ou/speed * std::sin(c_pi - opsi + alpha - beta));
          coll_cone[1] = alpha + beta  + std::asin(ou/speed * std::sin(c_pi - opsi + alpha + beta));

          // Here we need to check if CA is necessary
          //! then we set turning dir iff we are startng CA

          if (!m_in_ca){
            m_turn_dir = Angles::normalizeRadian(course-coll_cone[0]) <= Angles::normalizeRadian(course - coll_cone[1]) ? 0 : 1;
          }
          m_in_ca = true;
        }


        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_os_received){
            m_heading.value = ts.los_angle;
          }
          else
          {
            // assuming course control here.
            shouldCA(state, ts.course);

            if (m_in_ca)
              m_heading.value = coll_cone[m_turn_dir];
            else
              m_heading.value = ts.los_angle;
          }

          if (ts.cc)
            m_heading.value = Angles::normalizeRadian(m_heading.value + state.psi - ts.course);

          dispatch(m_heading);
}


// TODO
// Implement a timeout for received obstacle measurements?

      };
    }
  }
}

DUNE_TASK
