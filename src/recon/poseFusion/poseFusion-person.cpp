#include "recon/poseFusion/poseFusion.h"


void ReconstructPerson( 
                        PersonPose3D &person,
                        skeleton_t skelType,
                        std::vector< Calibration > calibs,
                        int   minInliersRANSAC,
                        float distThreshRANSAC
                      )
{
	int numJoints = SkelNumJoints( skelType );
	
	std::vector<bool> doneJoint( numJoints, false );
	for( unsigned jc = 0; jc < numJoints; ++jc )
	{
		// we might have already reconstructed this joint as part of a paired joint reconstruction.
		if( doneJoint[jc] )
			continue;
		
		int pairedJoint = IsPair( jc, skelType );
		if( pairedJoint >= 0 )
		{
			ReconstructJointPair( calibs, skelType, jc, pairedJoint, person );
			doneJoint[jc] = true;
			doneJoint[pairedJoint] = true;
		}
		else
		{
			ReconstructSingleJoint( calibs, jc, distThreshRANSAC, minInliersRANSAC, person );
			doneJoint[jc] = true;
		}
	}
}
