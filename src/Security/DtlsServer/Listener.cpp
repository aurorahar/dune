//***************************************************************************
// Copyright 2007-2021 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Ricardo Martins                                                  *
//***************************************************************************

#ifndef TRANSPORTS_DTLS_LISTENER_CPP_INCLUDED_
#define TRANSPORTS_DTLS_LISTENER_CPP_INCLUDED_

// ISO C++ 98 headers.
#include <map>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "ContactTable.hpp"
#include "LimitedComms.hpp"
#include "Node.hpp"
#include "Listener.hpp"

//mbedtls headers.
#include "mbedtls/entropy.h"
#include "mbedtls/build_info.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_cookie.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/timing.h"

namespace Security
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author lea
  namespace DtlsServer
  {
    using DUNE_NAMESPACES;

    //forward declaration of class Node
    // class Node;

    Listener::Listener(Tasks::Task& task, Security::DtlsServer::Node& node,
              float contact_timeout, bool trace):
      m_task(task),
      m_node(node),
      m_trace(trace),
      m_contacts(contact_timeout)
    {  }

    void
    Listener::getContacts(std::vector<Contact>& list)
    {
      m_contacts.getContacts(list);
    }

    void
    Listener::lockContacts(void)
    {
      m_contacts_lock.lockRead();
    }

    void
    Listener::unlockContacts(void)
    {
      m_contacts_lock.unlock();
    }

    void
    Listener::run(void)
    {
      Address addr;
      //todo: define variable for buffer size
      unsigned char bfr[1024];
      // double poll_tout = c_poll_tout / 1000.0;
      int ret, len;

      //todo: maybe keep the original poll


      while (!isStopping())
      {
        // try
        // {
          

          //todo: poll
          /*
          * 6. Read the echo Request
          */
          printf( "  < Read from client:" );
          fflush( stdout );

          // len = sizeof( bfr ) - 1;
          // memset( bfr, 0, sizeof( bfr ) );
          ret = 0;
          
          
          // ret = m_node.read(bfr, 512);
          
          // do ret = mbedtls_ssl_read( m_ssl, bfr, len );
          // while( ret == MBEDTLS_ERR_SSL_WANT_READ ||
          //       ret == MBEDTLS_ERR_SSL_WANT_WRITE );
          // Delay::wait(1);
         

          if( ret <= 0 )
          {
              switch( ret )
              {
                  case MBEDTLS_ERR_SSL_TIMEOUT:
                      m_task.err( " timeout\n\n" );
                      nodeState == 1;
                    //  m_node.reset();
                      break;

                  case MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY:
                      m_task.war( " connection was closed gracefully\n" );
                      nodeState = 2;
                      // stop();
                      break;

                  default:
                      m_task.err( " mbedtls_ssl_read returned -0x%x\n\n", (unsigned int) -ret );
                      // goto reset;
                      nodeState == 1;
                      // m_node.reset();
                      break;
              }
          }

          len = ret;

          
          printf( " %d bytes read in listener\n\n%s\n\n", len, bfr );


          // if (ret > 0)
          // {
          //   IMC::Message* msg = IMC::Packet::deserialize(bfr, ret);
  
          //   m_contacts_lock.lockWrite();
          //   m_contacts.update(msg->getSource(), addr);
          //   m_contacts_lock.unlock();

          //   m_task.dispatch(msg, DF_KEEP_TIME | DF_KEEP_SRC_EID);

          //   delete msg;

          // }
        // }
        // catch (std::exception & e)
        // {
        //   printf("error while unpacking message: %s",e.what());
        // }
      }

      
    }
  }
}

#endif