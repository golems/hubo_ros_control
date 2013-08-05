#include <hubo_trajectory_interface/hubo_trajectory.hpp>
#include <iostream>
#include <fstream>

using namespace Hubo;
using std::cout;
using std::endl;

void Hubo::print_vector( Vector vect )
{
    for(int i=0;i<vect.size();i++)
    {
        cout << vect[i] << " ";
    }
    cout << endl;
}

Trajectory::Trajectory()
{

}

//! Loads a trajectory from a file,
//! each line specifies a configuration
bool Trajectory::load_from_file( const std::string& filename, double frequency )
{
    std::vector<Vector> values;

    std::ifstream in( filename.c_str(), std::ios::in );
    if (!in){
        cout << "file " << filename << " does not exist" << endl;
        return false;
    }

    std::string line;

    while( std::getline( in, line ) )
    {
        Vector vect;
        std::stringstream ss( line );
        std::string chunk;

        while( std::getline( ss, chunk, ' ' ) )
        {
            double val;
            if( !Hubo::convert_text_to_num<double>( val, chunk, std::dec ) )
            {
                cout << "conversion from text failed" << endl;
                return false;
            }
            vect.push_back( val );
        }

        if( vect.size() > 0 )
        {
            values.push_back( vect );
        }
    }

    if( !values.empty() )
    {
        set_milestones_from_path( values, 1/frequency );
        return true;
    }
    else
    {
        milestones_.clear();
        return false;
    }
}

//! Sets the trajectory time law as equally spaced milestones
void Trajectory::set_milestones_from_path( const std::vector<Vector>& path, double dt )
{
    milestones_.resize( path.size() );

    double t = 0.0;
    for(int i=0;i<int(path.size());i++)
    {
        Milestone q;
        q.first = t;
        q.second = path[i];
        milestones_[i] = q;
        t += dt;
    }
}

//! Empties trajectory
void Trajectory::clear()
{
    milestones_.clear();
}


//! Adds a milestone at the end of the tajectory
void Trajectory::push_back( const Milestone& q )
{
    milestones_.push_back( q );
}

//! Gets the length of the trajcetory
double Trajectory::get_length() const
{
    if( milestones_.empty() )
    {
        return 0.0;
    }

    return milestones_.back().first;
}

//! Returns the configuration at time t
Vector Trajectory::get_config_at_time( double t ) const
{
    if( milestones_.empty() ) {
        cout << "Empty trajectory" << endl;
        return Vector(1,0.0);
    }
    if( milestones_.size() == 1 )
        return milestones_[0].second;

    if( t > milestones_.back().first )
    {
        return milestones_.back().second;
    }
    int i=0;
    for( int j=0;j<int(milestones_.size()); j++ )
    {
        if( t < milestones_[j].first )
        {
            i = j; break;
        }
    }

    if( i==0 )
    {
        return milestones_[0].second;
    }

    int p=i-1;
    double u=(t-milestones_[p].first)/(milestones_[i].first-milestones_[p].first);

    if( u < 0 || u > 1 )
    {
        cout << "Error in get config at time" << endl;
        return Vector(1,0.0);
    }

    return interpolate( milestones_[p].second, milestones_[i].second, u );
}

//! Interpolates linearly two configurations
Vector Trajectory::interpolate( const Vector& a, const Vector& b, double u ) const
{
    Vector out;
    if( a.size() != b.size() )
    {
        cout << "Error in interpolate" << endl;
        return out;
    }

    out = a;
    for( int i=0;i<int(out.size());i++)
    {
        out[i] += u*(b[i]-a[i]);
    }
    return out;
}
