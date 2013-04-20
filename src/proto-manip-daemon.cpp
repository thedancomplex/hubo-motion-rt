/*

    Please make note: This is only a prototype for a
    manipulator daemon and is meant for demonstration
    purposes ONLY. The final version will be a robust
    state machine which monitors the arms, handles
    manipulation-related tasks, and makes sure that
    everything behaves sanely. 

*/

/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Feb 03, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */



#include "manip.h"


/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_state;    // Feed-Back (State)


int main( int argc, char **argv )
{

    /* Open Ach Channel */
    int rr = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == rr );

    rr = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == rr );



    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_state H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));

    /* for size check */
    size_t fs;

    /* Get the current feed-forward (state) */
    rr = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != rr) {
        assert( sizeof(H_state) == fs );
    }




    H_ref.ref[RSP] = 0.1;
    ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));


    Hubo_Control hubo("proto-manip-daemon");
    H_ref.ref[RSP] = 0.2;
    ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
    ach_channel_t chan_manip_cmd;

    int r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    hubo_manip_cmd manip;
    memset( &manip, 0, sizeof(manip) );

    hubo.update();

    

    Eigen::Isometry3d Br, Bl;
    Vector6d right, left, zeros; zeros.setZero();
    Vector3d rtrans, ltrans, langles, rangles;
    

    hubo.getRightArmAngles(right);
    hubo.getLeftArmAngles(left);

    hubo.huboArmFK( Br, right, RIGHT );
    hubo.huboArmFK( Bl, left, LEFT );
    
    std::cout << "Performed initial FK" << std::endl;

    for(int i=0; i<3; i++)
    {
        manip.translation[RIGHT][i] = Br(i,3);
        manip.translation[LEFT][i] = Bl(i,3);
    }

    std::cout << "Putting first transformation" << std::endl;
    ach_put( &chan_manip_cmd, &manip, sizeof(manip) );

//    size_t fs;

    std::cout << "About to start loop" << std::endl;

    while( !daemon_sig_quit )
    {
        hubo.update();
        ach_get( &chan_manip_cmd, &manip, sizeof(manip), &fs, NULL, ACH_O_LAST );
        
        for(int i=0; i<3; i++)
        {
            rtrans(i) = manip.translation[RIGHT][i];
            ltrans(i) = manip.translation[LEFT][i];
            rangles(i) = manip.eulerAngles[RIGHT][i];
            langles(i) = manip.eulerAngles[LEFT][i];
        }

        // Handle the right arm
        Br = Eigen::Matrix4d::Identity();
        Br.translate(rtrans);
        Br.rotate( Eigen::AngleAxisd(rangles(0), Vector3d(1,0,0)) );
        Br.rotate( Eigen::AngleAxisd(rangles(1), Vector3d(0,1,0)) );
        Br.rotate( Eigen::AngleAxisd(rangles(2), Vector3d(0,0,1)) );
        hubo.huboArmIK( right, Br, zeros, RIGHT );
        hubo.setRightArmAngles( right );

        // Handle the left arm
        Bl = Eigen::Matrix4d::Identity();
        Bl.translate(ltrans);
        Bl.rotate( Eigen::AngleAxisd(langles(0), Vector3d(1,0,0)) );
        Bl.rotate( Eigen::AngleAxisd(langles(1), Vector3d(0,1,0)) );
        Bl.rotate( Eigen::AngleAxisd(langles(2), Vector3d(0,0,1)) ); 
        hubo.huboArmIK( left, Bl, zeros, LEFT );
        hubo.setLeftArmAngles( left );

	right[0] = 0.4;

        H_ref.ref[RSP] = right[0];
        H_ref.ref[RSR] = right[1];
        H_ref.ref[RSY] = right[2];
        H_ref.ref[REB] = right[3];
        H_ref.ref[RWY] = right[4];
        H_ref.ref[RWP] = right[5];

        /* Write to the feed-forward channel */
        ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
        // Send commands off to the control daemon
//        hubo.sendControls();
    }
    

    ach_close( &chan_manip_cmd );




}
