#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include<windows.h> 
#include <bits/stdc++.h>
//#include "matplotlibcpp.h"
//#define EIGEN_RUNTIME_NO_MALLOC 1



#define CORRESPONDENCES_SIZE 120        /* 120 (int)*/ // -> 120 * 2 * (x, y)
#define SCAN_SIZE 360                   /* 360 * 2 = 720 (double)*/
const int TEST_DIMENSION =2;
const int TEST_SIZE = 360;
double PI = 3.1415;

/*      INFOS ON INPUT

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
/*      OPERATIONS
 

*/

using namespace Eigen;
using namespace std;


Eigen::Matrix<double, 3, 3> getTransformation(double dx, double dy, double theta)
{
    Matrix3d T;
    T << cos(theta), -sin(theta), dx,
         sin(theta), cos(theta), dy,
         0,          0,          1;

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

void print_vec(double* vec, int size){
    for(int i =0; i< size; i++){
        std::cout << vec[i] << ", ";
    }
    std::cout << "\n";
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
    Eigen::Vector2d d = q - p;
    return (d.dot(d));
}

// most updated
Eigen::Matrix<int, Dynamic, 2> FindCorrenpondences_PtP(Eigen::Matrix<double, Dynamic, 2> PCL_o, Eigen::Matrix<double, Dynamic, 2> PCL_target){
//	const size = PCL_o.lines();
    Eigen::Matrix<int, TEST_SIZE, 2> corr;
    //corr.resize(0,0);
    //corr = {};
    int count = 0;
    double dists[TEST_SIZE] = {0}, sum_dists =0, med_dists, std_dev_dists, variance_dists, var1;
    // iterate over pcl_o
    double acceptance_level = 5.5;
    for (int i =0; i < PCL_o.rows(); i++){
        // para cada ponto no conjunto Origem encontrar qual o segmento de linha cuja a distancia euclidiana seja minima
        // alinhar para esquerda
        // correspondencia do ponto
        double minDIST = 9999, dist =9999;
        int closestPointIndex = -1;
        Eigen::Vector2d q= {PCL_o(i,0), PCL_o(i,1)};
        //Eigen::Vector2d q_aux= {PCL_o(i-1,0), PCL_o(i-1,1)};
        Eigen::Vector2d p1;
        for (int j = 0; j < PCL_target.rows() ; j++){
            p1= {PCL_target(j,0), PCL_target(j,1)};
            //Eigen::Vector2d p2= {PCL_target(j+1,0), PCL_target(j+1,1)};
            //Eigen::Vector2d q= {PCL_o(i,0), PCL_o(i,1)};
            dist = DistancePtP(p1, q);
            //  (PCL_o(i,0)!=0 && PCL_target(j,0)!= 0)
            if(dist < minDIST){                     // nooo zeros  to test
                minDIST = dist;
                closestPointIndex = j;              // alinha direita
            }
        }
        // adiciona correspondencia


        dists[count] = minDIST;
        sum_dists += minDIST;				// use to calc error later

        if(minDIST == 9999){
            minDIST = 9999;
            std::cout << "FODEU " << dist << "\t" << p1 << "\t" << q << std::endl;
            //break;
        }
        
        // Implementar alguma abordagem para remover os pontos que possuem distancias grandes entre indices consecutivos
        // ELIMINATE PAIR BY THE GIVEN THRESHOLD (acceptance_level)
        if(minDIST < acceptance_level){												        // try to eliminate pairs with error
			corr.row(count) << i, closestPointIndex;
			count++;
		}else{
			corr.row(count) << -1, -1;												        // eliminate pairs with error
			count++;
		}
//		variance_dists *= 0.008333;
		// DESVIO PADRAO
    }

	// NEWWW CODE
	// MEDIA DAS DISTANCIAS
	// med_dists = sum_dists * (0.002777);        // med = sum * 1/360
	// var1 = 0;
	// for (int i =0 ; i < corr.rows() ; i++){
	// 	// VARIANCIA
	// 	var1 += abs(dists[i] - med_dists);

	// }
	// float med_deviation = var1*0.002777;

	// for (int i =0; i < corr.rows() ; i++){										  //	eliminate on median deviation
	// 	// VARIANCIA
	// 	variance_dists = abs(dists[i] - med_dists);
	// 	if(variance_dists > med_deviation*1.9){
	// 		corr.row(i) << -1, -1;
	// 	}
	// }


	//  TEST BY PAIRS IF THEY ARE PAIRED SIMILARLY IN THE OTHER SURFACE
//	int radius = 3;
//	for (int i =2; i < corr.rows()-4 ; i+=2){
//		if(corr(i,0) !=-1 && corr(i+1,0) !=-1){
//			int a = corr(i,1), b = corr(i+1, 1);
//			if(abs(a-b) > radius){						// SE FOR MAIOR QUE O RAIO ESTABELECIDO REMOVE O PAR C MAIOR DISTANCIA
//
//				corr.row(i) << -1, -1;
//				corr.row(i+1) << -1, -1;
//
//			}
//		}else{
//			if(corr(i,0) !=-1){										// SE FOR PAR ISOLADO
//				corr.row(i) << -1, -1;
//			}else if(corr(i+1,0) !=-1){
//				corr.row(i) << -1, -1;
//			}
//		}
//	}
//    print_vec(dists, 360);
	return corr;

}


Eigen::Matrix<double,1,2> CenterOfMass( Eigen::MatrixXd PtCld){
    double xU=0., yU=0.;
    int ct=0;
    for (int i = 0; i < PtCld.rows(); i++){
        xU += PtCld(i,0);
        yU += PtCld(i,1);

    }
    xU =  xU /  PtCld.rows();
    yU =  yU /  PtCld.rows();
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
        if(s != -1) err += sqrt( pow(tgt(s2,0) - org(s,0), 2) + pow(tgt(s2,1) - org(s,1), 2)); 

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

    int siz = 1;
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

    Eigen::Matrix<double,1,3> 				out, transf;                  						    // update  de paramentros da transformação
    Eigen::Matrix<double,1,2> 				translation, translationR, centroid_p, centroid_q;      // palpite inicial
    Eigen::Matrix<double, 3, 3> 			tt;                                                     // transformação
    Eigen::Matrix<int, Dynamic, 2> 			correspondences, n_corr;         						// correspondencias - DECLARE STATIC
    Eigen::Matrix<double, Dynamic, 2> 		covariance;                                             // not used
    Eigen::Matrix<double, TEST_SIZE*2, 3> 	A;                    									// Matriz A         - DECLARE STATIC
    Eigen::Matrix<double, TEST_SIZE*2, 1> 	b;                    									// Matriz b         - DECLARE STATIC
//    double prev_err=0, new_err=0;
    double err = 0;
    transf.setZero();
    //transf.block<1,2>(0,0) = CenterOfMass(tgt) - CenterOfMass(org);

    //correspondences = FindCorrenpondences(org, tgt);        // corr. antes do palplite por centroide

    //prev_err = getError(org,tgt, correspondences);          // erro antes do palpite inicial


    // centroid_p = CenterOfMass(org);
    // centroid_q = CenterOfMass(tgt);
    // translation = centroid_q - centroid_p;    // translação levando em conta o centroide das duas nuvens
    // transf.block<1,2>(0,0) = translation;


    //std::cout << "translation before: \t" << translation << "\n";
    //org = computeTransform(org, transf);                    // initial guess

    //correspondences = FindCorrenpondences_PtP(org, tgt);        // correspondencias
    correspondences = FindCorrenpondences_PtP(org, tgt);
    //std::cout << "corr:\n" << correspondences << std::endl;
    //n_corr = FilterCorr(org, tgt, correspondences);
    n_corr = correspondences;
    //err = getError(org, tgt, correspondences);              // erro após a transformação inicial
    //std::cout << "NEWW _ corr:\n" << n_corr << std::endl;
    int index_system=0;
    for( int i=0; i < n_corr.rows(); i++){

        int s  = n_corr(i,0);
        int s2 = n_corr(i,1);

//        covariance.conservativeResize(i*2 +2, 2);
//        A.conservativeResize(i*2 +2, 3);                // remove
//        b.conservativeResize(i*2 +2, 1);                // remove

        if(s!=-1){																							// if not dropped correspondences



	//        covariance.row(2*i) = org.row(s) - centroid_p;
	//        covariance.row(2*i +1) = tgt.row(s2) - centroid_q;

			A.row(2*index_system)      <<  1 , 0 , -org(s,1);              //  A = | 1    0   -p_y |
			A.row(2*index_system +1)   <<  0 , 1 ,  org(s,0);              //      | 0    1    p_x |

			b.row(2*index_system)      <<  tgt(s2,0) - org(s,0);           //  b = | q_x - p_x |
			b.row(2*index_system +1)   <<  tgt(s2,1) - org(s,1);           //      | q_q - p_y |
			index_system++;
        }
        // else{
        //     A.row(2*index_system)      <<  0,0,0;                //  A = | 1    0   -p_y |
		// 	A.row(2*index_system +1)   <<  0,0,0;                   //      | 0    1    p_x |

		// 	b.row(2*index_system)      <<  0;                       //  b = | q_x - p_x |
		// 	b.row(2*index_system +1)   <<  0;                       //      | q_q - p_y |
		// 	index_system++;
        // }

    }




    //translation = CenterOfMass(tgt) - CenterOfMass(org);

    //out = A.jacobiSvd(ComputeFullU | ComputeFullV).solve(b);

    out = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    //out.block<1,2>(0,0)

    //out.block<1,2>(0,0) << 0,0;
    tt = getTransformation(0, 0, out(0,2));

    translationR = centroid_q.transpose() - tt.block<2,2>(0,0) * centroid_p.transpose();         // Translation = Cq - R.Cp

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
    return out; //+ transf;
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
        //if (i % 13 ==0){
        //    source(i,0) = 0;
        //    source(i,1) = 0;
        //}
    }
    return source;
}

Eigen::Matrix<double, Dynamic, 2> setCornerWnoise(Eigen::Matrix<double, Dynamic, 2> source){
    /*
           /\
          /  \
         /    \
        /      \
    */
    int ct=1;
    for (int i =0; i < source.rows() ; i++){
        if(i> source.rows()/2){
            source(i,0) = i + ((double)rand()) / RAND_MAX;
            source(i,1) = source.rows() -i + ((double)rand()) / RAND_MAX;
            ct++;
        }else{
            source(i,0) = i + ((double)rand()) / RAND_MAX;
            source(i,1) = i + ((double)rand()) / RAND_MAX;
        }
        //if (i % 13 ==0){
        //    source(i,0) = 0;
        //    source(i,1) = 0;
        //}
    }
    return source;
}

Eigen::Matrix<double, 120, 2> drop_points_120(Eigen::Matrix<double, 360, 2> scan_2d){

	Eigen::Matrix<double, 120, 2> out;
	int idx_out = 0;

	for(int i =2; i < 360; i+=3){
//		if( scan_2d(i,0) == 0.  || scan_2d(i,1) == 0.){
//			for(int j = i; j < 360; j++){
//				if( scan_2d(j,0) != 0. && scan_2d(j,1) != 0.){
//					out(idx_out,0) = scan_2d(j, 0);
//					out(idx_out,1) = scan_2d(j, 1);
//					idx_out++;
//					j=361;
//				}
//			}
//		}else{
//			out(idx_out,0) = scan_2d(i, 0);
//			out(idx_out,1) = scan_2d(i, 1);
//			idx_out++;
//		}
		out(idx_out, 0) = scan_2d(i, 0);
		out(idx_out,1) = scan_2d(i, 1);
		idx_out++;
		if(idx_out == 119){
			idx_out = 119;
		}
	}

	return out;
}


Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> f_scan_i_ToEigenMatrix(double* scan_ranges, int size_scan){
	double ang;
	//double coord[720];
	Eigen::Matrix<double, TEST_SIZE, TEST_DIMENSION> out;
    double conversionToCentimeter = 0.1;


	for ( int i=0 ; i< size_scan ; i++){
		ang = i * PI * (0.01745);        // convertion to radians
//		coordinates[i][0] =  scan_ranges[i] * cos(ang);
//		coordinates[i][1] =  scan_ranges[i] * sin(ang);
		out(i,0) = double(scan_ranges[i] * cos(ang))  * conversionToCentimeter;
		out(i,1) = double(scan_ranges[i] * sin(ang))  * conversionToCentimeter;
        //std::cout << scan_ranges[i] << std::endl;
	}
    //Sleep(10000);
	return out;
}


const int num_of_samples = 100;

Eigen::MatrixXd introduceError(Eigen::MatrixXd in, double size){
    Eigen::MatrixXd out;
    out.conservativeResize(in.rows(), 2);   
    for (int i =0; i < in.rows(); i++){
        out(i,0) = in(i,0) + size * (((double)rand()) / RAND_MAX);
        out(i,1) = in(i,1) + size * (((double)rand()) / RAND_MAX);
    }

    return out;
}

double** fileToDistances(std::string path, int size, double** out){
    //Eigen::Matrix<double, num_of_samples, TEST_SIZE> out;
    std::string see, line;
    
    std::fstream sensor_file(path);
    int count_scan_idx=0;
    std::cout  << path << std::endl;
    while(sensor_file.is_open() &&  count_scan_idx < size && getline(sensor_file, line) ){
        int  count_dist_idx = 0;
        for (int i =0; i < line.length(); i++){
            //std::cout << see << std::endl;                                                                    // it gets here! numbers OK
            if(line[i] != '[' && line[i] != ' ' && line[i] != ',' && line[i] != ']'){
                see += line[i];

            }else if(line[i] == ','){
                if(see.find("inf") != string::npos){
                    // is found
                    out[count_scan_idx][count_dist_idx] = -1;
                    //std::cout  << out[count_scan_idx][count_dist_idx] << std::endl;                               //
                    count_dist_idx +=1;
                }else{
                    out[count_scan_idx][count_dist_idx] = stod(see);
                    //std::cout  << out[count_scan_idx][count_dist_idx] << std::endl;                               //
                    count_dist_idx +=1;
                    //std::cout << stod(see) << std::endl;                                                          // it gets here! OK, 
                }
    
                see = "";                                                                                           // but always check size of vector
            }else if(line[i] == ']'){
                count_scan_idx += 1;
                count_dist_idx = 0;
                //Sleep(10000);
            }
            //if( count_scan_idx == 99) break;
        }
        //Sleep(5000);
    }
    //for(int i =0 ; i < 100; i++){
    //    print_vec(out[i], TEST_SIZE);
    //}
    sensor_file.close();
    return out;
}


void EigentoVec(Eigen::MatrixXd eigenM, double** vec, int lin, int col){
    for (int i =0; i < lin; i++){
        for(int j =0; j < col; i++){
            vec[i][j] = eigenM(i, j); 
        }
    }
}

int mainCPP(){

    //fileToSamplePoints2D("./data/scan001.txt");


    // PC TEST
    //const int TEST_DIMENSION = 2;
    //const int TEST_SIZE = 60;
    double error=0;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> source_points, updt_p, rot_points;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> transformed_points;
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> target_points;
    //source_points = Eigen::Matrix<double, TEST_SIZE, TEST_DIMENSION>::Random();
    source_points = setCornerWnoise(source_points);
    Eigen::Matrix<double, 1, 3> transf, translation;                                              // palpite inicial
    Eigen::Matrix<double, 1, 3> true_transform, icp_transform, cumulative, final_transform;
    Eigen::Matrix<double, 1, 2> centroid_a, centroid_b, res;
    Eigen::Matrix<int, Dynamic, 2> CORR;
    Eigen::Matrix<double, 3, 3> HOM;
    cumulative.setZero();
    transf.setZero();
    true_transform << .52, .45, 0.00134;
    transformed_points = computeTransform(source_points, true_transform);

    //std::cout << "target: \n"           << transformed_points << "\n";
    transformed_points = introduceError(transformed_points, 0.55);
    //std::cout << "target(noise): \n"           << transformed_points << "\n";

    //transf.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(source_points);

    //std::cout << "origin: \n" << source_points << "\n";
    //std::cout << "target: \n" << transformed_points << "\n";




    //std::cout << "MASS org:\n" << CenterOfMass(source_points) << std::endl;
    //std::cout << "MASS tgt:\n" << CenterOfMass(transformed_points) << std::endl;

    
    // INITIAL GUESS


    updt_p = source_points;                                                             // variavel temporaria recebe scan anterior

    double err = 0, previous_err = 0, diff = 999;

    for (int i =0; i < 100; i++){
        //std::cout << "error: \t" << "\n";
        CORR = FindCorrenpondences_PtP(updt_p, transformed_points);                 
        //n_err = getError(updt_p, transformed_points, CORR );                      // 
        
        //std::cout << "error: \t" << "\n";
        icp_transform = ICP(updt_p, transformed_points, 100, 0.005);
        //std::cout << "error: \t" << "\n";
     


        //cumulative += icp_transform;
        updt_p = computeTransform(updt_p, icp_transform);                           // aplicar transformação nos pontos temp
        cumulative += icp_transform;
        //std::cout << "error: \t" << "\n";
        //CORR = FindCorrenpondences_PtP(updt_p,transformed_points);                
        err = getError(updt_p, transformed_points, CORR);
        diff = previous_err - err;
        previous_err = err;
        std::cout << "iteracao atual   : \t"            <<  i                       << "\t";
        std::cout << "diff error: \t"                   <<  diff                    << "\t";
        std::cout << "Transformacao esperada: \t"       <<  true_transform          << "\t";
        std::cout << "Transformacao atual   : \t"       <<  icp_transform(0,0)      << " "  << icp_transform(0,1) << " " << cumulative(0,2) << "\n";
        if(err < 1e-15){
            std::cout << "iteration (out): \t" <<  i << "\n";
            break;
        }

        //std::cout << "Transformation [i=" << i << "]:\t" << icp_transform << std::endl;
    }

    final_transform << 0, 0, cumulative(0,2);
    //final_transform << cumulative;

    rot_points = computeTransform(source_points, final_transform);

    std::cout << "ROT AUX:[ "<< final_transform(0,2) << " ] \n" << rot_points << "\n";


    final_transform.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(rot_points);

    std::cout << "Centroid Target:\n"                   << CenterOfMass(transformed_points)         << std::endl;
    std::cout << "Centroid TEMP:\n"                     << CenterOfMass(rot_points)                 << std::endl;
    std::cout << "Translation FINAL ICP:\n"             << final_transform                          << std::endl;

    rot_points = computeTransform(source_points, final_transform);

    //std::cout << "ROT AUX (2): \n"      << rot_points             << "\n";

    //std::cout << "origin: \n"           << source_points          << "\n";
    //std::cout << "target: \n"           << transformed_points     << "\n";
    //std::cout << "output: \n"           << updt_p                 << "\n";
    //std::cout << "target: \n"           << transformed_points << "\n";
    return 0;
}

Eigen::Matrix<double, 1, 3> main_ICP(       Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> source_points, 
                                            Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION> target_points        ){
    // PC TEST

    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION>  updt_p, rot_points, transformed_points;

    Eigen::Matrix<double, 1, 3> transf, translation;                                            // palpite inicial
    Eigen::Matrix<double, 1, 3> true_transform, icp_transform, cumulative, final_transform;     // Transformações (tx, ty, theta)
    Eigen::Matrix<double, 1, 2> centroid_a, centroid_b, res;                                    // Centros de massa
    Eigen::Matrix<int, Dynamic, 2> CORR;                                                        // Guarda Correspondencias
    Eigen::Matrix<double, 3, 3> HOM;
    // INICIALIZAÇAO
    cumulative.setZero();                       // ZEROS   
    transf.setZero();                           // ZEROS
    true_transform << 0.25, 10.4, -0.584;       // Somente para teste unitário
    transformed_points = target_points;         // used variable = transformed_points


 
    
    // INITIAL GUESS

    transf.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(source_points);
    updt_p = computeTransform(source_points, transf);
    double err = 0., n_err = 0., diff =0., previous_err = 0.;
    translation = transf;
    centroid_b = CenterOfMass(transformed_points);


    for (int i =0; i < 100; i++){
        std::cout << updt_p << "\n";
        CORR = FindCorrenpondences_PtP(updt_p, transformed_points);
        //n_err = getError(updt_p, transformed_points, CORR);
        
        //std::cout  <<  "error: \t" << "\n";
        icp_transform = ICP(updt_p, transformed_points, 100, 0.005);
        //std::cout  <<  "error: \t" << "\n";
     


        //cumulative += icp_transform;
        updt_p = computeTransform(updt_p, icp_transform);               // n 
        cumulative += icp_transform;
        //std::cout << "error: \t" << "\n";
        //CORR = FindCorrenpondences_PtP(updt_p,transformed_points);
        //err = getError(updt_p, transformed_points, CORR );              // n

        //std::cout << "CORRESPONDENCES: \t" << CORR << "\n";
        //std::cout << "errorss: \t" << n_err << "\t" << err << "\n";
        err = getError(updt_p, transformed_points, CORR);
        diff = previous_err - err;
        previous_err = err;
        if(i > 6 && err <= 0.0005){
            std::cout << "error: \t" <<  err << "\n";
            std::cout << "iteration (out): \t" <<  i << "\n";
            break;
        }
        //std::cout << "Transformation [i=" << i << "]:\t" << icp_transform << std::endl;
    }

    final_transform << 0, 0, cumulative(0,2);
    rot_points = computeTransform(source_points, final_transform);
    
    final_transform.block<1,2>(0,0) = CenterOfMass(transformed_points) - CenterOfMass(rot_points);

    //std::cout << "Transformation [i=100]:\n" << cumulative << std::endl;
    //std::cout << "Transformation INITIAL [i=100]:\n" << transf << std::endl;
    std::cout << "Translation FINAL ICP:\n" << final_transform << std::endl;

    transf << cumulative(0,0)/2, cumulative(0,1)*cos(cumulative(0,2)), cumulative(0,2);
    //std::cout << "Transformation FUKET [i=100]:\n" << transf << std::endl;
    //std::cout << "origin: \n" << source_points << "\n";
    source_points = computeTransform(source_points, cumulative);
    //std::cout << "output: \n" << updt_p << "\n";
    //std::cout << "target: \n" << transformed_points << "\n";
    return final_transform;
}

int size_samples = 200;
double** f_scans= (double**)malloc(size_samples * sizeof(double*));


int main(){
    //int size_samples = 100;
    //double** f_scans= (double**)malloc(100 * sizeof(double*));

    for (int i =0; i< size_samples; i++){
        f_scans[i] = (double*)malloc(TEST_SIZE * sizeof(double));
    }

    f_scans = fileToDistances("./data/scanLASTEST01.txt", size_samples, f_scans);
    //print_vec(f_scans[45], TEST_SIZE);
    Eigen::Matrix<double, TEST_SIZE , TEST_DIMENSION>  org, tgt;
    Eigen::Matrix<double, 1, 3> trans_steps;
    ofstream outFile("./data/PC_ICP_output5.txt");
    for (int i = 0; i < size_samples - 3; i+=3){
        org << f_scan_i_ToEigenMatrix(f_scans[i], TEST_SIZE);
        tgt << f_scan_i_ToEigenMatrix(f_scans[i+3], TEST_SIZE);
        //std::cout << tgt << std::endl;
        //print_vec(f_scans[i], TEST_SIZE);
        //std::cout << org << std::endl;
        //Sleep(2000);
        //std::cout << f_scans[i] << std::endl;
        trans_steps = main_ICP(org, tgt);
        std::cout << trans_steps << std::endl;
        outFile << trans_steps << std::endl;
    }
    outFile.close();
}


// int main(){
//     srand(time(0));
//     mainCPP();
// }