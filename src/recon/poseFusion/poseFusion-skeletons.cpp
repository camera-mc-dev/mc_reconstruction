#include "recon/poseFusion/poseFusion.h"

bool IsLeft( int jc, skeleton_t skel )
{
	if( skel == SKEL_OPOSE )
	{
		switch(jc)
		{
			case 18:
			case 16:
			case 5:
			case 6:
			case 7:
			case 12:
			case 13:
			case 14:
			case 21:
			case 19:
			case 20:
				return true;
				break;
		}
		return false;
	}
	else if( skel == SKEL_APOSE )
	{
		switch(jc)
		{
			case 17:
			case 15:
			case 5:
			case 6:
			case 7:
			case 11:
			case 12:
			case 13:
				return true;
				break;
		}
		return false;
	}
	else if( skel == SKEL_DLCUT )
	{
		switch(jc)
		{
			case 3:
			case 4:
			case 5:
			case 9:
			case 10:
			case 11:
				return true;
				break;
		}
		return false;
	}
	return false;
}

//
// For a given joint, return the opposite joint if it has one.
// i.e. if jc == left elbow, return jc of right elbow
//
int IsPair( int jc, skeleton_t skel )
{
	if( skel == SKEL_OPOSE )
	{
		switch(jc)
		{
			case 17: return 18; break;   case 18: return 17; break;
			case 15: return 16; break;   case 16: return 15; break;
			
			case  5:  return 2; break;   case  2:  return 5; break;
			case  6:  return 3; break;   case  3:  return 6; break;
			case  7:  return 4; break;   case  4:  return 7; break;
			
			case 12:  return 9; break;   case  9: return 12; break;
			case 13: return 10; break;   case 10: return 13; break;
			case 14: return 11; break;   case 11: return 14; break;
			
			case 19: return 22; break;   case 22:  return 19; break;
			case 20: return 23; break;   case 23:  return 20; break;
			case 21: return 24; break;   case 24:  return 21; break;
			
		}
		return -1;
	}
	else if( skel == SKEL_APOSE )
	{
		switch(jc)
		{
			case 16: return 17; break;   case 17: return 16; break;
			case 14: return 15; break;   case 15: return 14; break;
			
			case  5:  return 2; break;   case  2:  return 5; break;
			case  6:  return 3; break;   case  3:  return 6; break;
			case  7:  return 4; break;   case  4:  return 7; break;
			
			case 11: return  8; break;   case  8: return 11; break;
			case 12: return  9; break;   case  9: return 12; break;
			case 13: return 10; break;   case 10: return 13; break;
		}
		return -1;
	}
	else if( skel == SKEL_DLCUT )
	{
//		// TODO!
// 		switch(jc)
// 		{
// 			
// 		}
		return -1;
	}
	return false;
}


int SkelNumJoints( skeleton_t skel )
{
	switch( skel )
	{
		case SKEL_OPOSE:
			return 25; 
		case SKEL_APOSE:
			return 18;
		case SKEL_DLCUT:
			return 14;
	}
}
