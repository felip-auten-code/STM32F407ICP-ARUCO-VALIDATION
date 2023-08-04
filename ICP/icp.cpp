#include <iostream>
#include <String>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

//#define EIGEN_RUNTIME_NO_MALLOC 1



#define CORRESPONDENCES_SIZE 100        /* 120 (int)*/ // -> 120 * 2 * (x, y)
#define SCAN_SIZE 360                   /* 360 * 2 = 720 (double)*/


/*

    SIZE_OF_VARIABLES:
        M: Matrix
            M_LINES_COLLUMNS

        V: Vector
            V_SIZE
    -------------------------

    -----FROM_LIDAR-----

    SCAN_P:
        M_360_2

    SCAN_Q:
        M_360_2

    SCAN_P_FILTERED?:

    SCAN_Q_FILTERED?:

    UPDATED_P:
        M_360_2


    CORRESPONDENCES TABLE:
        M_120_2  ?? or M_360_2
    
    -----LINEAR_LEAST_SQR-----
    
    Matrix A:
        M_240_2
    Matrix b:
        M_240_1

    -----FOR_OPERATIONS-----

    CENTROID_P:
        V_2
    CENTROID_Q:
        V_2


    
    _____RESULT______
    


    CURRENT TRANSFORM:
        V_3 // M_1_3                                    (is used transposed in operations)
    


*/
/*
    OPERATIONS




    #problematic
        computeTransform  --> more mem space ?  what is temporary ?

*/

using namespace Eigen;
using namespace std;




Eigen::Vector2d PolarToCartesian2D(const Eigen::Vector2d &ranges){

    Eigen::Vector2d r = {0,0};
    return r;
}

Eigen::Matrix<double, 3, 3> getTransformation(double dx, double dy, double theta)
{
    Matrix3d T;
    T << cos(theta), -sin(theta), dx,
         sin(theta), cos(theta), dy,
         0,          0,          1;
//    T << cos(theta),  sin(theta),  dx,
//             -sin(theta), cos(theta), dy,
//             0,           0,          1;
//
    if(T.determinant() < 0){
        T << cos(theta),  sin(theta),  dx,
             -sin(theta), cos(theta), dy,
             0,           0,          1;
    }
    return T;
}

double mod2d(Eigen::Vector2d in){
    return sqrt(pow(in(0), 2) + pow(in(1), 2));
}

double mod3d2(Eigen::Vector3d in){
    return sqrt(pow(in(0), 2) + pow(in(1), 2) + pow(in(2), 2));
}


Eigen::Vector2d UnitVec(Eigen::Vector2d u){
    //double mod_u = mod2d(u);
    return u / u.dot(u);
}

double ScalarProj(Eigen::Vector2d u, Eigen::Vector2d v){
    double mag = v.dot(u) / u.dot(u);
    return mag;
}

Eigen::Vector2d Proj_V_in_U(Eigen::Vector2d u, Eigen::Vector2d v){
    Eigen::Vector2d proj;
    proj = UnitVec(u) * ScalarProj(u, v);
    return proj;
}

double DistancePointToLineSegment2D_(    Eigen::Vector2d p1, 
                                        Eigen::Vector2d p2, 
                                        Eigen::Vector2d q ){

    Eigen::Vector2d vec_p1_p2 = p2 - p1;
    Eigen::Vector2d vec_p1_q = q - p1;
    Eigen::Vector2d proj, d, cross;

    proj = Proj_V_in_U(vec_p1_p2 ,vec_p1_q);

    d = vec_p1_q - proj;
    //std::cout << "dist: \t" << mod2d(d) << "\n";
    return mod2d(d) * mod2d(vec_p1_q); 

}

double DistancePointToLineSegment2D(    Eigen::Vector2d &p1, 
                                        Eigen::Vector2d &p2, 
                                        Eigen::Vector2d &q ){
    Eigen::Vector2d vec_p1_p2 = p2 - p1;
    Eigen::Vector2d vec_p1_q = q - p1;
    Eigen::Vector3d vec1, vec2, cross;
    vec1 << vec_p1_p2(0), vec_p1_p2(1), 0;
    vec2 << vec_p1_q(0), vec_p1_q(1), 0;
    cross = vec1.cross(vec2);

    double mag = mod3d2(cross) / mod2d(vec_p1_p2);         
    return mag;
}



Eigen::Matrix<int, Dynamic, 2> FindCorrenpondences(Eigen::Matrix<double, Dynamic, 2> PCL_o, Eigen::Matrix<double, Dynamic, 2> PCL_target){
    Eigen::Matrix<int, Dynamic, 2> corr;
    //corr.resize(0,0);
    //corr = {};
    int count =0;
    // iterate over pcl_o
    for (int i =0; i< PCL_o.rows(); i++){
        // para cada ponto no conjunto Origem encontrar qual o segmento de linha cuja a distancia euclidiana seja minima
        // alinhar para esquerda
        // correspondencia do ponto 
        double minDIST = 9999;
        int closestPointIndex = -1;
        for (int j =0; j < PCL_target.rows() -1; j++){
            Eigen::Vector2d p1= {PCL_target(j,0), PCL_target(j,1)};
            Eigen::Vector2d p2= {PCL_target(j+1,0), PCL_target(j+1,1)};
            Eigen::Vector2d q= {PCL_o(i,0), PCL_o(i,1)};
            double dist = DistancePointToLineSegment2D_(p1, p2, q);
            if(dist < minDIST){
                minDIST = dist;
                closestPointIndex = j+1;      // alinha direita
            }
        }
        // adiciona correspondencia
        if(closestPointIndex >= 0){
            //corr.push_back({PCL_o(i,0), PCL_o(i,1), PCL_target(closestPointIndex,0), PCL_target(closestPointIndex,1)});
            corr.conservativeResize(count+1, 2);
            //corr.row(count) << PCL_o(i,0), PCL_o(i,1), PCL_target(closestPointIndex,0), PCL_target(closestPointIndex,1);
            corr.row(count) << i, closestPointIndex;
            count++;
        }
    }

    return corr;
}

double DistancePtP(Eigen::Vector2d p, Eigen::Vector2d q){
    Eigen::Vector2d dist = q - p;
    return dist.dot(dist);
}

Eigen::Matrix<int, Dynamic, 2> FindCorrenpondences_PtP(Eigen::Matrix<double, Dynamic, 2> PCL_o, Eigen::Matrix<double, Dynamic, 2> PCL_target){
    Eigen::Matrix<int, Dynamic, 2> corr;
    //corr.resize(0,0);
    //corr = {};
    int count =0;
    // iterate over pcl_o
    for (int i =0; i< PCL_o.rows(); i++){
        // para cada ponto no conjunto Origem encontrar qual o segmento de linha cuja a distancia euclidiana seja minima
        // alinhar para esquerda
        // correspondencia do ponto 
        double minDIST = 9999;
        int closestPointIndex = -1;
        for (int j =0; j < PCL_target.rows() ; j++){
            Eigen::Vector2d p1= {PCL_target(j,0), PCL_target(j,1)};
            //Eigen::Vector2d p2= {PCL_target(j+1,0), PCL_target(j+1,1)};
            Eigen::Vector2d q= {PCL_o(i,0), PCL_o(i,1)};
            double dist = DistancePtP(p1, q);
            if(dist < minDIST){
                minDIST = dist;
                closestPointIndex = j;      // alinha direita
            }
        }
        // adiciona correspondencia
        if(closestPointIndex >= 0){
            //corr.push_back({PCL_o(i,0), PCL_o(i,1), PCL_target(closestPointIndex,0), PCL_target(closestPointIndex,1)});
            corr.conservativeResize(count+1, 2);
            //corr.row(count) << PCL_o(i,0), PCL_o(i,1), PCL_target(closestPointIndex,0), PCL_target(closestPointIndex,1);
            corr.row(count) << i, closestPointIndex;
            count++;
        }
    }

    return corr;
}


Eigen::MatrixXd DisposeOutOfRange(  const Eigen::MatrixXd &PtCld1, 
                                    const Eigen::MatrixXd &PtCld2 ){


    Eigen::Vector2d r = {0,0};
    return r;
}

Eigen::Matrix<double,1,2> CenterOfMass( Eigen::MatrixXd PtCld){
    double xU=0, yU=0;
    int ct=0;
    for (int i = 0; i < PtCld.rows(); i++){
        xU += PtCld(i,0);
        yU += PtCld(i,1);
        ct++;
    }
    xU = xU / ct;
    yU = yU / ct;
    Eigen::Matrix<double,1,2> r ;
    r << xU, yU;
    //r(0) = xU;
    //r(1) = yU;
    return r;
}

Eigen::Matrix<double, Dynamic, 2> computeTransform( Eigen::Matrix<double, Dynamic, 2> source,
                                                    Eigen::Matrix<double, 1, 3> transform){

    int siz = source.rows();
    Eigen::Matrix<double, Dynamic, 2> out;
    Eigen::MatrixXd temp;
    Eigen::Vector2d tt;
    Eigen::Matrix<double, 3, 3> transformation; 
    transformation = getTransformation(transform(0,0), transform(0,1), transform(0,2));
    for (int i=0; i < source.rows(); i++){
        tt = source.row(i); 
        //out.resize(i+1, 2);
        out.conservativeResize(i+1,2);
        //out(i,0) = source(i,0);
        //out(i,1) = source(i,1);
        //out.resize(i+2, 2);
        //out.row(i) = (transformation.block<2,2>(0,0) * source.row(i).transpose() + transformation.block(2,1,0,2).transpose());
        temp = (transformation.block<2,2>(0,0) * source.row(i).transpose() + transformation.block<2,1>(0,2));
        //std::cout << "temp \n\n" << tt << std::endl;
        //out.block<1,2>(i,0) = temp.transpose();
        //out.row(i) = temp.transpose();
        //std::cout << "iteration: \t " << i << std::endl;
        //std::cout << "" << temp << std::endl;
        out.row(i) = temp.transpose();
    }

    return out;                           
} 

double getError(Eigen::Matrix<double, Dynamic, 2> org, Eigen::Matrix<double, Dynamic, 2> tgt, Eigen::Matrix<int, Dynamic, 2> correspondences){
    double err=0;
    for(int i=0; i < correspondences.rows(); i++){
        int s = correspondences(i,0);
        int s2 = correspondences(i,1);
        err += sqrt(pow(tgt(s2,0) - org(s,0), 2) + pow(tgt(s2,1) - org(s,1), 2)); 

    }
    return err;
}


Eigen::Matrix<int, Dynamic, 2> FilterCorr(Eigen::Matrix<double, Dynamic, 2> org, Eigen::Matrix<double, Dynamic, 2> tgt, Eigen::Matrix<int, Dynamic, 2> correspondences){
    Eigen::Matrix<double, 1, 2>  err, centroid_p, controid_q;
    Eigen::Matrix<int, Dynamic, 2> new_corr;
    double mean_err = 0;
    for(int i =0; i < correspondences.rows(); i++){
        int s = correspondences(i, 0);
        int s2 = correspondences(i, 1);
        mean_err += mod2d(org.row(s) - tgt.row(s2));
    }
    mean_err /= correspondences.rows();

    int siz =1;
    for(int i =0; i < correspondences.rows(); i++){
        int s = correspondences(i, 0);
        int s2 = correspondences(i, 1);


        if( mod2d(org.row(s) - tgt.row(s2)) <= mean_err ){           // 10% tolerance
            new_corr.conservativeResize(siz, 2);
            new_corr.row(siz-1) << s, s2;
            siz++;
        }
    }

    return new_corr;
}

Eigen::Matrix<double,1,3> ICP(  Eigen::Matrix<double, Dynamic, 2> org, 
                                Eigen::Matrix<double, Dynamic, 2> tgt, 
                                int MAX_ITERATIONS, 
                                double TOLERANCE = 5e-3){

    Eigen::Matrix<double,1,3> out, transf;                  // update  de paramentros da transformação
    Eigen::Matrix<double,1,2> translation, translationR, centroid_p, centroid_q;                  // palpite inicial
    Eigen::Matrix<double, 3, 3> tt;
    Eigen::Matrix<int, Dynamic, 2> correspondences, n_corr;         // correspondencias - DECLARE STATIC
    Eigen::Matrix<double, Dynamic, 2> covariance; 
    Eigen::Matrix<double, Dynamic, 3> A;                    // Matriz A         - DECLARE STATIC
    Eigen::Matrix<double, Dynamic, 1> b;                    // Matriz b         - DECLARE STATIC
    double prev_err=0, new_err=0, err=0;
    transf.setZero();
    //transf.block<1,2>(0,0) = CenterOfMass(tgt) - CenterOfMass(org);
    
    //correspondences = FindCorrenpondences(org, tgt);        // corr. antes do palplite por centroide
    
    //prev_err = getError(org,tgt, correspondences);          // erro antes do palpite inicial
    centroid_p = CenterOfMass(org);
    centroid_q = CenterOfMass(tgt);
    translation = centroid_q - centroid_p;    // translação levando em conta o centroide das duas nuvens
    transf.block<1,2>(0,0) = translation;
    //std::cout << "translation before: \t" << translation << "\n";
    //org = computeTransform(org, transf);                    // initial guess
    
    //correspondences = FindCorrenpondences_PtP(org, tgt);        // correspondencias 
    correspondences = FindCorrenpondences_PtP(org, tgt);  
    //std::cout << "corr:\n" << correspondences << std::endl;
    //n_corr = FilterCorr(org, tgt, correspondences);
    n_corr = correspondences;
    //err = getError(org, tgt, correspondences);              // erro após a transformação inicial
    //std::cout << "NEWW _ corr:\n" << n_corr << std::endl;
    for( int i=0; i< n_corr.rows(); i++){
        
        int s = n_corr(i,0);
        int s2 = n_corr(i,1);
         
        covariance.conservativeResize(i*2 +2, 2);
        A.conservativeResize(i*2 +2, 3);                // remove
        b.conservativeResize(i*2 +2, 1);                // remove

        
        err += pow(tgt(s2,0) - org(s,0), 2) + pow(tgt(s2,1) - org(s,1), 2);


        covariance.row(2*i) = org.row(s) - centroid_p;
        covariance.row(2*i +1) = tgt.row(s2) - centroid_q;

        A.row(2*i)      <<  1 , 0 , -org(s,1);              //  A = | 1    0   -p_y |
        A.row(2*i +1)   <<  0 , 1 ,  org(s,0);              //      | 0    1    p_x |

        b.row(2*i)      <<  tgt(s2,0) - org(s,0);           //  b = | q_x - p_x |
        b.row(2*i +1)   <<  tgt(s2,1) - org(s,1);           //      | q_q - p_y |

    }
        

    

    //translation = CenterOfMass(tgt) - CenterOfMass(org); 

    //out = A.jacobiSvd(ComputeFullU | ComputeFullV).solve(b);

    out = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    //out.block<1,2>(0,0)

    //out.block<1,2>(0,0) << 0,0;
    tt = getTransformation(0, 0, out(0,2));

    translationR = centroid_q.transpose() - tt.block<2,2>(0,0) * centroid_p.transpose();
    //translationR = out.block<1,2>(0,0).transpose() - tt.block<2,2>(0,0) * out.block<1,2>(0,0).transpose();

    //new_err = getError(org, tgt, correspondences);
    out.block<1,2>(0,0) = translationR;
    //out.block<1,2>(0,0) << tx, ty;


    //std::cout << "MASS A: \t" << centroid_p << "\n";
    //std::cout << "MASS B: \t" << centroid_q << "\n";
    //std::cout << "ROT: \n" << tt.block<2,2>(0,0) << "\n";
    //std::cout << "TRNAF: \n" << tt.block<2,2>(0,0) << "\t" << out(0,2) << "\n";
    //std::cout << "translationR: \t" << translationR << "\n";
    //std::cout << "transLATE: \t" << translation << "\n";
    return out;//+ transf;
}

Eigen::Matrix<double, Dynamic, 2> setLine(Eigen::Matrix<double, Dynamic, 2> source){
    /*
           /
          /  
         /    
        /     
    */
    int ct=1;
    for (int i =0; i < source.rows() ; i++){

        source(i,0) = i;
        source(i,1) = i;
        

    }
    return source;
}


Eigen::Matrix<double, Dynamic, 2> setCorner(Eigen::Matrix<double, Dynamic, 2> source){
    /*
           /\
          /  \
         /    \
        /      \
    */
    int ct=1;
    for (int i =0; i < source.rows() ; i++){
        if(i> source.rows()/2){
            source(i,0) = i;
            source(i,1) = source.rows() -i;
            ct++;
        }else{
            source(i,0) = i;
            source(i,1) = i;
        }

    }
    return source;
}

int main(){

    // PC TEST
    const int TEST_DIMENSION = 2;
    const int TEST_SIZE = 60;
    double error=0;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> source_points, updt_p, rot_points;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> transformed_points;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> target_points;
    source_points = Eigen::Matrix<double, TEST_SIZE, TEST_DIMENSION>::Random();
    source_points = setLine(source_points);
    Eigen::Matrix<double,1,3> transf, translation;                       // palpite inicial
    Eigen::Matrix<double, 1, 3> true_transform, icp_transform, cumulative, final_transform;
    Eigen::Matrix<double, 1, 2> centroid_a, centroid_b, res;
    Eigen::Matrix<int, Dynamic, 2> CORR;
    Eigen::Matrix<double, 3, 3> HOM;
    cumulative.setZero();
    transf.setZero();
    true_transform << 0.25, 10.4, 0.584;
    transformed_points = computeTransform(source_points, true_transform);


    //transf.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(source_points);

    std::cout << "origin: \n" << source_points << "\n";
    std::cout << "target: \n" << transformed_points << "\n";
    //std::cout << "cos: \t" << cos(0) << "\n";



    std::cout << "MASS org:\n" << CenterOfMass(source_points) << std::endl;
    std::cout << "MASS tgt:\n" << CenterOfMass(transformed_points) << std::endl;

    //source_points = computeTransform(source_points, transf);  
    
    // INITIAL GUESS

    transf.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(source_points);
    updt_p = computeTransform(source_points, transf);
    double err = 0, n_err =0;
    translation = transf;
    centroid_b = CenterOfMass(transformed_points);


    for (int i =0; i< 8; i++){
        CORR = FindCorrenpondences_PtP(updt_p,transformed_points);
        n_err = getError(updt_p, transformed_points, CORR );
        

        icp_transform = ICP(updt_p, transformed_points, 100, 0.005);

     


        //cumulative += icp_transform;
        updt_p = computeTransform(updt_p, icp_transform);
        cumulative += icp_transform;

        CORR = FindCorrenpondences_PtP(updt_p,transformed_points);
        err = getError(updt_p, transformed_points, CORR );


        std::cout << "error: \t" << err << "\n";

        std::cout << "Transformation [i=" << i << "]:\t" << icp_transform << std::endl;
    }

    final_transform << 0, 0, cumulative(0,2);
    rot_points = computeTransform(source_points, final_transform);
    
    final_transform.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(rot_points);

    std::cout << "Transformation [i=100]:\n" << cumulative << std::endl;
    std::cout << "Transformation INITIAL [i=100]:\n" << transf << std::endl;
    std::cout << "Translation FINAL ICP:\n" << final_transform << std::endl;

    transf << cumulative(0,0)/2, cumulative(0,1)*cos(cumulative(0,2)), cumulative(0,2);
    //std::cout << "Transformation FUKET [i=100]:\n" << transf << std::endl;
    //std::cout << "origin: \n" << source_points << "\n";
    source_points = computeTransform(source_points, cumulative);
    //std::cout << "output: \n" << updt_p << "\n";
    //std::cout << "target: \n" << transformed_points << "\n";
    return 0;
}