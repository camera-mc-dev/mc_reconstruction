#include "recon/poseFusion/poseFusion.h"
#include "commonConfig/commonConfig.h"


Skeleton::Skeleton(  )
{
}

Skeleton::Skeleton( std::string cfgFile )
{
	try
	{
		cout << "parsing skeleton config: " << cfgFile << endl;
		libconfig::Config cfg;
		cfg.readFile( cfgFile.c_str() );
		
		libconfig::Setting &ptsSetting  = cfg.lookup("points");
		libconfig::Setting &hierSetting = cfg.lookup("hierarchy");
		
		for( unsigned c = 0; c < ptsSetting.getLength(); ++c )
		{
			libconfig::Setting &grp = ptsSetting[c];
			
			if( grp.getLength() == 1 )
			{
				SkelPoint p;
				p.name   = (const char*)grp[0][0];
				p.idx    = grp[0][1];
				p.isLeft = false;
				p.pair   = -1;
				
				data[ p.idx ] = p;
				id2Name[ p.idx ] = p.name;
				name2Id[ p.name ] = p.idx;
			}
			else if( grp.getLength() == 2 )
			{
				SkelPoint r, l;
				
				// right always first. That's our rule.
				r.name   = (const char*)grp[0][0];
				r.idx    = grp[0][1];
				r.isLeft = false;
				
				l.name   = (const char*)grp[1][0];
				l.idx    = grp[1][1];
				l.isLeft = true;
				
				
				r.pair = l.idx;
				l.pair = r.idx;
				
				
				data[ r.idx ] = r;
				id2Name[ r.idx ] = r.name;
				name2Id[ r.name ] = r.idx;
				
				data[ l.idx ] = l;
				id2Name[ l.idx ] = l.name;
				name2Id[ l.name ] = l.idx;
			}
			else
			{
				throw( std::runtime_error( "skeleton points must be single or a left/right pair" ) );
			}
		}
		
		
		for( unsigned c = 0; c < hierSetting.getLength(); ++c )
		{
			std::string a = (const char*) hierSetting[c][0];
			std::string b = (const char*) hierSetting[c][1];
			
			parents[a] = b;
		}
		
	}
	catch( libconfig::SettingException &e)
	{
		cout << "Setting error: " << endl;
		cout << e.what() << endl;
		cout << e.getPath() << endl;
		exit(0);
	}
	catch( libconfig::ParseException &e )
	{
		cout << "Parse error:" << endl;
		cout << e.what() << endl;
		cout << e.getError() << endl;
		cout << e.getFile() << endl;
		cout << e.getLine() << endl;
		exit(0);
	}
}






bool Skeleton::IsLeft( unsigned jc )
{
	// TODO: CHECK input exists!
	return data[jc].isLeft;
}

bool Skeleton::IsLeft( std::string name )
{
	// TODO: CHECK input exists!
	return data[ name2Id[name] ].isLeft;
}

//
// For a given joint, return the opposite joint if it has one.
// i.e. if jc == left elbow, return jc of right elbow
//
int Skeleton::IsPair( unsigned jc )
{
	return data[jc].pair;
}

std::string Skeleton::IsPair( std::string name )
{
	return id2Name[ IsPair( name2Id[name] ) ];
}


unsigned Skeleton::GetNumKeypoints( )
{
	return data.size();
}

std::string Skeleton::GetName( unsigned kpc )
{
	// TODO: CHECK input exists!
	return id2Name[kpc];
}

std::map< int, std::string > Skeleton::GetNames()
{
	std::map< int, std::string > ret;
	
	for( auto i = data.begin(); i != data.end(); ++i )
	{
		ret[ i->second.idx ] = i->second.name;
	}
	
	return ret;
}

unsigned    Skeleton::GetIdx( std::string name )
{
	// TODO: CHECK input exists!
	return name2Id[ name ];
}

std::string Skeleton::GetParent( std::string name )
{
	// TODO: CHECK input exists!
	return parents[ name ];
}

unsigned    Skeleton::GetParent( unsigned kpc     )
{
	// TODO: CHECK input exists!
	return name2Id[  parents[ id2Name[kpc] ] ];
}
