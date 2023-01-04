#include "recon/poseFusion/poseFusion.h"


void ReconstructPerson( 
                        PersonPose3D &person,
                        Skeleton skeleton,
                        std::vector< Calibration > calibs,
                        float minConf,
                        leftRightDecision_t lrd,
                        int   minInliersRANSAC,
                        float distThreshRANSAC
                      )
{
	int numJoints = skeleton.GetNumKeypoints();
	
	std::vector<bool> doneJoint( numJoints, false );
	for( unsigned jc = 0; jc < numJoints; ++jc )
	{
// 		if( jc != 4 )
// 			continue;
		
		// we might have already reconstructed this joint as part of a paired joint reconstruction.
		if( doneJoint[jc] )
			continue;
		
		int pairedJoint = skeleton.IsPair( jc );
		if( pairedJoint >= 0 )
		{
			ReconstructJointPair( calibs, skeleton, jc, pairedJoint, minConf, lrd, person );
			doneJoint[jc] = true;
			doneJoint[pairedJoint] = true;
			
			if( std::isnan( person.joints[jc](0) ) || std::isnan(person.joints[jc](1)) || std::isnan(person.joints[jc](2) ) )
			{
				cout << "damn a " << jc << endl;
				exit(0);
			}
			
			if( std::isnan( person.joints[pairedJoint](0) ) || std::isnan(person.joints[pairedJoint](1)) || std::isnan(person.joints[pairedJoint](2) ) )
			{
				cout << "damn b " << pairedJoint << endl;
				exit(0);
			}
		}
		else
		{
			ReconstructSingleJoint( calibs, jc, minConf, distThreshRANSAC, minInliersRANSAC, person );
			if( std::isnan( person.joints[jc](0) ) || std::isnan(person.joints[jc](1)) || std::isnan(person.joints[jc](2) ) )
			{
				cout << "damn " << jc << endl;
				person.joints[jc] << 0,0,0,0;
			}
			doneJoint[jc] = true;
		}
	}
}
