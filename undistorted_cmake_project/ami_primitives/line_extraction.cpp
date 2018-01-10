/*
Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
*/


#ifndef AMI_DLL_CPP
#define AMI_DLL_CPP
#endif

#include "line_extraction.h"
#include "../ami_lens_distortion/lens_distortion_procedures.h"
#include "../ami_utilities/utilities.h"

//THIS FUNCTION CORRECTS THE ORIENTATION OF THE EDGE USING THE DIVISION MODEL.
//PARAMETERS: THE EDGE POSITION, THE ORIENTATION (SIN,COS) AND THE LENS
//            DISTORTION MODEL
vector<float> orientation_update_quotient(point2d<double> p, float seno,
										  float coseno,
										  lens_distortion_model ldm)
{
	vector<float> corrected_orientation;
	double a,b,norma;
	//POINT PLUS THE ORIENTATION
	point2d<double> p_ori(p.x + coseno, p.y + seno);
	//WE APPLY THE MODEL TO THE ORIGINAL POINT
	point2d<double> p_prime(ldm.evaluation_quotient(p));
	//WE APPLY THE MODEL TO THE POINT PLUS THE ORIENTATION
	point2d<double> p_ori_prime(ldm.evaluation_quotient(p_ori));

	//WE COMPUTE THE NEW ORIENTATION AND THE NORM
	a = p_ori_prime.x - p_prime.x;
	b = p_ori_prime.y - p_prime.y;
	norma = sqrt(a*a + b*b);

	if((a*a + b*b) <= 0)
	{
		corrected_orientation.push_back(coseno);
		corrected_orientation.push_back(seno);
		return corrected_orientation;
	}
	else
	{
		corrected_orientation.push_back(a/norma);
		corrected_orientation.push_back(b/norma);
		return corrected_orientation;
	}
}

//------------------------------------------------------------------------------

/**
* \fn double line_extraction::
line_equation_distortion_extraction_improved_hough_quotient(
const subpixel_image_contours &subpixel_contours,
image_primitives &image_primitive,
float distance_point_line_max, int nlineas=13,
float angle_resolution=0.25,
float distance_resolution=2.,
float initial_distortion_parameter=-0.1,
float final_distortion_parameter=0.1,
float distortion_parameter_resolution=0.01,
float angle_point_orientation_max_difference=15.,
bool lens_distortion_estimation=true)
* \brief Computation of lines using an improved version of Hough which includes
1 parameter lens distortion division model
* \author Luis Alvarez
*/
double line_equation_distortion_extraction_improved_hough_quotient(
	const subpixel_image_contours &subpixel_contours /** INPUT CONTOUR SUBPIXEL
													 INFORMATION */,
													 image_primitives &image_primitive /** OUTPUT IMAGE PRIMITIVES WHERE LINES AND
																					   DISTORTION MODEL IS DEFINED */,
																					   float distance_point_line_max /** INPUT MAX DISTANCE ALLOWED BETWEEN POINTS
																													 AND ASSOCIATED LINES */,
																													 int nlineas /** INPUT NUMBER OF LINES TO RETURN */,
																													 float angle_resolution /** INPUT ANGLE RESOLUTION */,
																													 float distance_resolution  /** INPUT DISTANCE RESOLUTION */,
																													 float initial_distortion_parameter /** INPUT INITIAL VALUE OF NORMALIZED
																																						DISTORTION PARAMETER */,
																																						float final_distortion_parameter /** INPUT FINAL VALUE NORMALIZED DISTORTION
																																														 PARAMETER */,
																																														 float distortion_parameter_resolution /** INPUT DISTORTION PARAMETER
																																																							   DISCRETIZATION STEP */,
																																																							   float angle_point_orientation_max_difference /** MAX DIFFERENCE (IN DEGREES)
																																																																			OF THE POINT ORIENTATION
																																																																			ANGLE AND THE LINE ANGLE */,
																																																																			bool lens_distortion_estimation /** BOOL TO CONTROL IF WE ESTIMATE THE LENS
																																																																											DISTORTION MODEL */,
																																																																											lens_distortion_model ini_ldm /** INITIAL DISTORTION MODEL */)
{
	int i,j,l,k,d,q;
	int width_score,height_score,depth_score;
	int width=subpixel_contours.get_width();
	int height=subpixel_contours.get_height();
	double x2,x3,xc=(double) width/2.;
	double y2,y3,yc=(double) height/2.;
	double best_distortion_parameter=0;
	float ***score;
	double *seno,*coseno;
	double ami_pi=acos((double) -1.);
	double distancia,max_score;
	int nlineas_plus=nlineas;
	if( lens_distortion_estimation==true) nlineas_plus=nlineas;
	point2d<double> dc(xc,yc);
	double dmi;

	// WE CHECK DISCRETIZATION PARAMETER VALUES: DISTANCE AND ANGLE RESOLUTION
	if(distance_resolution<=0 || angle_resolution<=0 ){
		//cout << "Problems in function line_equation_distortion_extraction_\
		//         improved_hough()" << endl;
		// cout << "Discrezation parameters lower or equal to 0 " << endl;
		// cout << "Introduce a number to continue" << endl;
		int val = scanf("%d",&i);
		if(val==EOF) {}//printf("Read error\n");
		return -1;
	}
	// WE DEFINE SCORE VOLUME SIZE
	width_score=(int) (2.*sqrt((double) width*width+height*height)/
		distance_resolution+2);
	height_score=(int) (180./angle_resolution);
	depth_score=distortion_parameter_resolution>0 ?
		(int)(1.+(final_distortion_parameter-initial_distortion_parameter)/
		distortion_parameter_resolution) : 1;
	{int ml,mk,mu;
	if ( depth_score > 0 && height_score > 0 && width_score > 0) {
		score=(float ***) malloc(sizeof(float **)*(depth_score));
		if (score == 0){ ; int val = scanf("%d",&ml); if(val==(-1)) ;} 
		score[0]=(float **)malloc(sizeof(float *)*(depth_score)*(width_score));
		if (score[0] == 0){free(score); score = 0; ; int val = scanf("%d",&ml);
		if(val==(-1)) ;}
		score[0][0]=(float *)malloc(sizeof(float)*(depth_score)*(width_score)*(height_score));
		if (score[0][0] == 0){free(score[0]); free(score); score = 0; ; int val = scanf("%d",&ml); if(val==(-1)) ;} 
		for(mu=0;mu<depth_score;mu++){
			for(ml=0;ml<(width_score);ml++) score[0][ml+mu*(width_score)]=&(score[0][0][ml*(height_score)+ mu*(height_score)*(width_score)]); 
		score[mu]=&(score[0][mu*(width_score)]);} 
		for(mu=0;mu<depth_score;mu++) 
			for(ml=0;ml<width_score;ml++) 
				for(mk=0;mk<height_score;mk++) 
					score[mu][ml][mk]=0;

	} else score = 0;};

	// WE DEFINE ORIENTATION DISCRETIZATION VECTOR
	seno   =(double*)malloc(sizeof(double)*height_score);
	coseno =(double*)malloc(sizeof(double)*height_score);
	double paso_angle=angle_resolution*ami_pi/180.;
	for(l=0;l<height_score;l++) {
		coseno[l]=cos((double) l*paso_angle);
		seno[l]=sin((double) l*paso_angle);
	}
	//············································································
	//UPDATE THE SUBPIXEL_CONTOURS OBJECT WITH THE INITIAL LENS DISTORTION MODEL
	subpixel_image_contours sp_contours_modifed(width, height);
	if(ini_ldm.get_d().size()<1)
	{
		//IF THE INITIAL MODEL IS EMPTY, THE MODIFIED MODEL HAS THE DEFAULT VALUE. 
		//WE ONLY COPY
		for(int i=0; i<width*height; i++)
		{
			sp_contours_modifed.get_x()[i]      = subpixel_contours.get_x()[i];
			sp_contours_modifed.get_y()[i]      = subpixel_contours.get_y()[i];
			sp_contours_modifed.get_coseno()[i] = subpixel_contours.get_coseno()[i];
			sp_contours_modifed.get_seno()[i]   = subpixel_contours.get_seno()[i];
			sp_contours_modifed.get_c()[i]      = subpixel_contours.get_c()[i];
		}
		dmi  = dc.norm();
		dmi *= dmi;
	}
	else
	{
		//ELSE, WE UPDATE THE SUBPIXEL_CONTOURS OBJECT USING THE PROVIDED MODEL
		for(int i=0; i<width*height; i++)
		{
			//CORRECT THE POSITION
			point2d<double> ori(subpixel_contours.get_x()[i],
				subpixel_contours.get_y()[i]);
			point2d<double> corrected(ini_ldm.evaluation_quotient(ori));
			sp_contours_modifed.get_x()[i] = corrected.x;
			sp_contours_modifed.get_y()[i] = corrected.y;

			//CORRECT THE ORIENTATION
			vector<float> corrected_orientation;
			double a,b,norma;
			//Point plus the orientation
			point2d<double> p_ori(ori.x + subpixel_contours.get_coseno()[i],
				ori.y + subpixel_contours.get_seno()[i]);
			//We apply the model to the original point
			point2d<double> p_prime(ini_ldm.evaluation_quotient(ori));
			//We apply the model to the point plus the orientation
			point2d<double> p_ori_prime(ini_ldm.evaluation_quotient(p_ori));

			//We compute the new orientation and the norm
			a = p_ori_prime.x - p_prime.x;
			b = p_ori_prime.y - p_prime.y;
			norma = sqrt(a*a + b*b);

			if((a*a + b*b) <= 0)
			{
				corrected_orientation.push_back(subpixel_contours.get_coseno()[i]);
				corrected_orientation.push_back(subpixel_contours.get_seno()[i]);
			}
			else
			{
				corrected_orientation.push_back(a/norma);
				corrected_orientation.push_back(b/norma);
			}
			//WE UPDATE THE VALUE OF THE SIN AND COS
			sp_contours_modifed.get_coseno()[i] = corrected_orientation[0];
			sp_contours_modifed.get_seno()[i]   = corrected_orientation[1];

			//WE UPDATE THE CONTOUR COMPONENT OF SUBPIXEL_CONTOURS OBJECT
			sp_contours_modifed.get_c()[i] = subpixel_contours.get_c()[i];
		}
		//WE UPDATE THE VALUE OF THE DISTORTION CENTER
		xc  = ini_ldm.get_distortion_center().x;
		yc  = ini_ldm.get_distortion_center().y;
		dc  = point2d<double>(xc,yc);
		dmi = dc.norm();
		dmi*= dmi;
	}
	//············································································
	double votation_score, max_votation_score = -1;

	// WE FILL SCORE VOLUME
#ifdef _OPENMP
#pragma omp parallel for private(votation_score,max_score,distancia,d,q,x2,\
	y2,x3,y3)
#endif
	for(int k=0; k<depth_score; k++)
	{
		//POINTER TO THE ACUTAL SLICE
		float **score_k = score[k];
		//THE ANGLE INTERVAL
		int angle;
		//WE COMPUTE THE LENS DISTORTION PARAMETER VALUE FOR EACH ITERATION
		double p = initial_distortion_parameter +
			(k*distortion_parameter_resolution);
		double kdistortion = -p / (dmi + dmi*p);
		//AND WE BUILD A MODEL WITH IT
		lens_distortion_model it_ldm;
		it_ldm.set_distortion_center(dc);
		it_ldm.get_d().resize(2);
		it_ldm.get_d()[0] = 1.;
		it_ldm.get_d()[1] = kdistortion;
		//FOR EACH PAIR (DISTANCE,ORIENTATION)
		for(int i=0; i<height; i++)
		{
			for(int j=0; j<width; j++)
			{
				q = i*width+j;
				//WE CHECK IF THE POINT IS A CONTOUR
				if(sp_contours_modifed.get_c()[q] == 0) continue;

				x3 = x2 = sp_contours_modifed.get_x()[q];
				y3 = y2 = sp_contours_modifed.get_y()[q];
				//WE EVALUEATE THE DISTORTION MODEL IN THE POINT
				if(k!=0)
				{
					point2d<double> res=it_ldm.evaluation_quotient(point2d<double>(x2,y2));
					x3 = res.x;
					y3 = res.y;
				}
				//ORIENTATION CORRECTION
				vector<float> corrected_orientation = orientation_update_quotient(
					point2d<double>(x2,y2),
					sp_contours_modifed.get_seno()[q],
					sp_contours_modifed.get_coseno()[q],
					it_ldm);
				//WE COMPUTE THE POINT ANGLE
				if(corrected_orientation[1]>=0)
				{
					angle = (int) ((ami_pi-atan2(corrected_orientation[1],
						corrected_orientation[0]))/paso_angle);
				}
				else
				{
					angle = (int) ((ami_pi-atan2(-corrected_orientation[1],
						-corrected_orientation[0]))/paso_angle);
				}
				if(angle==height_score) angle=height_score-1;

				//WE ESTIMATE DE ANGLE INTERVAL
				int l_min=(int) (angle-angle_point_orientation_max_difference/
					angle_resolution);
				int l_max=(int) (angle+angle_point_orientation_max_difference/
					angle_resolution);
				int id=2;//Neighbourhood
				//WE VOTE FROM THE MINIMUM VALUE OF THE ANGLE INTERVAL TO THE POINT ANGLE
				//ACCORDING TO THE DISTANCE FROM THE POINT TO THE LINE
				for(int l=l_min;l<(int) angle;l++)
				{
					if(l<0)
					{
						distancia=-coseno[height_score+l]*y3-seno[height_score+l]*x3;
						for(int nd=-id;nd<=id;nd++)
						{
							d= width_score/2+((int)(distancia/distance_resolution+0.5))+nd;
							if(d>=0 && d<width_score)
							{
								double distancia_recta = fabs((distancia+
									coseno[height_score+l]*y3+
									seno[height_score+l]*x3)+
									nd*distance_resolution);
								score_k[d][height_score+l]+=1./(1.+distancia_recta);
							}
						}
					}
					else
					{
						distancia=-coseno[l]*y3-seno[l]*x3;
						for(int nd=-id;nd<=id;nd++)
						{
							d= width_score/2+((int)(distancia/distance_resolution+0.5))+nd;
							if(d>=0 && d<width_score)
							{
								double distancia_recta = fabs((distancia+coseno[l]*y3+
									seno[l]*x3)+
									nd*distance_resolution);
								score_k[d][l]+=1./(1.+distancia_recta);
							}
						}
					}
				}
				//AND VOTE FROM THE POINT ANGLE TO THE MAXIMUM VALUE OF THE ANGLE INTERVAL
				for(int l=(int) angle;l<l_max;l++)
				{
					if(l>=height_score)
					{
						distancia=-coseno[l-height_score]*y3-seno[l-height_score]*x3;
						for(int nd=-id;nd<=id;nd++)
						{
							d=width_score/2+((int)(distancia/distance_resolution+0.5))+nd;
							if(d>=0 && d<width_score)
							{
								double distancia_recta = fabs((distancia+
									coseno[l-height_score]*y3-
									seno[l-height_score]*x3)+
									nd*distance_resolution);
								score_k[d][l-height_score]+=1./(1.+distancia_recta);
							}
						}
					}
					else
					{
						distancia=-coseno[l]*y3-seno[l]*x3;
						for(int nd=-id;nd<=id;nd++)
						{
							d=width_score/2+((int)(distancia/distance_resolution+0.5))+nd;
							if(d>=0 && d<width_score)
							{
								double distancia_recta = fabs((distancia+coseno[l]*y3+
									seno[l]*x3)+
									nd*distance_resolution);
								score_k[d][l]+=1./(1.+distancia_recta);
							}
						}
					}
				}
			} //END J LOOP
		} //END I LOOP
		//··········································································
		//WE SELECT THE MAXIMUM OF THE SLICE
		votation_score = 0.;
		vector<int> m(nlineas_plus);
		vector<int> n(nlineas_plus);
		vector<int> m_max(nlineas_plus);
		vector<int> n_max(nlineas_plus);
		vector<line_points> lines(nlineas_plus);
		float first_max_score = 0;

		for(int l=0;l<nlineas_plus;l++)
		{
			max_score=0;
			//WE SELECT A LOCAL MAXIMUM INSIDE THE SLICE (WITH A NEIGBOURHOOD OF ONE)
			for(int i=0;i<height_score;i++)
			{
				for(int j=0;j<width_score;j++)
				{
					//GENERAL CASE (INSIDE THE IMAGE)
					if(i>0 && j>0 && i<height_score-1 && j<width_score-1)
					{
						if(score_k[j][i]>=max_score &&
							score_k[j][i]!=first_max_score &&
							score_k[j][i]>=score_k[j-1][i-1] &&
							score_k[j][i]>=score_k[j][i-1] &&
							score_k[j][i]>=score_k[j+1][i-1] &&
							score_k[j][i]>=score_k[j-1][i] &&
							score_k[j][i]>=score_k[j+1][i] &&
							score_k[j][i]>=score_k[j-1][i+1] &&
							score_k[j][i]>=score_k[j][i+1] &&
							score_k[j][i]>=score_k[j+1][i+1])
						{
							max_score = score_k[j][i];
							m[l] = i; n[l] = j;
						}
					}
					else
					{
						//CORNERS
						//LEFT-UP
						if(i==0 && j==0)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[width_score-1][height_score-1] &&
								score_k[j][i]>=score_k[j][height_score-1] &&
								score_k[j][i]>=score_k[j+1][height_score-1] &&
								score_k[j][i]>=score_k[width_score-1][i] &&
								score_k[j][i]>=score_k[j+1][i] &&
								score_k[j][i]>=score_k[width_score-1][i+1] &&
								score_k[j][i]>=score_k[j][i+1] &&
								score_k[j][i]>=score_k[j+1][i+1])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//RIGHT-UP
						if(i==0 && j==width_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[j-1][height_score-1] &&
								score_k[j][i]>=score_k[j][height_score-1] &&
								score_k[j][i]>=score_k[0][height_score-1] &&
								score_k[j][i]>=score_k[j-1][i] &&
								score_k[j][i]>=score_k[0][i] &&
								score_k[j][i]>=score_k[j-1][i+1] &&
								score_k[j][i]>=score_k[j][i+1] &&
								score_k[j][i]>=score_k[0][i+1])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//LEFT-DOWN
						if(i==height_score-1 && j==0)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[width_score-1][i-1] &&
								score_k[j][i]>=score_k[j][i-1] &&
								score_k[j][i]>=score_k[j+1][i-1] &&
								score_k[j][i]>=score_k[width_score-1][i] &&
								score_k[j][i]>=score_k[j+1][i] &&
								score_k[j][i]>=score_k[width_score-1][0] &&
								score_k[j][i]>=score_k[j][0] &&
								score_k[j][i]>=score_k[j+1][0])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//RIGHT-DOWN
						if(i==height_score-1 && j==width_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[j-1][i-1] &&
								score_k[j][i]>=score_k[j][i-1] &&
								score_k[j][i]>=score_k[0][i-1] &&
								score_k[j][i]>=score_k[j-1][i] &&
								score_k[j][i]>=score_k[0][i] &&
								score_k[j][i]>=score_k[j-1][0] &&
								score_k[j][i]>=score_k[j][0] &&
								score_k[j][i]>=score_k[0][0])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//TOP
						if(i==0 && j>0 && j<width_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[j-1][height_score-1] &&
								score_k[j][i]>=score_k[j][height_score-1] &&
								score_k[j][i]>=score_k[j+1][height_score-1] &&
								score_k[j][i]>=score_k[j-1][i] &&
								score_k[j][i]>=score_k[j+1][i] &&
								score_k[j][i]>=score_k[j-1][i+1] &&
								score_k[j][i]>=score_k[j][i+1] &&
								score_k[j][i]>=score_k[j+1][i+1])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//BOTTOM
						if(i==height_score-1 && j>0 && j<width_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[j-1][i-1] &&
								score_k[j][i]>=score_k[j][i-1] &&
								score_k[j][i]>=score_k[j+1][i-1] &&
								score_k[j][i]>=score_k[j-1][i] &&
								score_k[j][i]>=score_k[j+1][i] &&
								score_k[j][i]>=score_k[j-1][0] &&
								score_k[j][i]>=score_k[j][0] &&
								score_k[j][i]>=score_k[j+1][0])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//LEFT
						if(j==0 && i>0 && i<height_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[width_score-1][i-1] &&
								score_k[j][i]>=score_k[j][i-1] &&
								score_k[j][i]>=score_k[j+1][i-1] &&
								score_k[j][i]>=score_k[width_score-1][i] &&
								score_k[j][i]>=score_k[j+1][i] &&
								score_k[j][i]>=score_k[width_score-1][i+1] &&
								score_k[j][i]>=score_k[j][i+1]
							&& score_k[j][i]>=score_k[j+1][i+1])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
						//RIGHT
						if(j==width_score-1 && i>0 && i<height_score-1)
						{
							if(score_k[j][i]>=max_score &&
								score_k[j][i]!=first_max_score &&
								score_k[j][i]>=score_k[j-1][i-1] &&
								score_k[j][i]>=score_k[j][i-1] &&
								score_k[j][i]>=score_k[0][i-1] &&
								score_k[j][i]>=score_k[j-1][i] &&
								score_k[j][i]>=score_k[0][i] &&
								score_k[j][i]>=score_k[j-1][i+1] &&
								score_k[j][i]>=score_k[j][i+1] &&
								score_k[j][i]>=score_k[0][i+1])
							{
								max_score = score_k[j][i];
								m[l] = i; n[l] = j;
							}
							continue;
						}
					}
				} //END J FOR
			} //END I FOR
			//WE SAVE THE LINE PARAMETERS
			lines[l].set_a((float)seno[m[l]]);
			lines[l].set_b((float)coseno[m[l]]);
			lines[l].set_c((float)(n[l]-width_score/2)*distance_resolution);
			//WHE INCREMENT THE VOTATION SCORE OF THE SLICE
			votation_score+=max_score;
			//········································································
			//WE TAKE THE FIRST MAXIMUM VALUE+0.01
			if(l==0)
				first_max_score = max_score+0.01;
			// AND WE SET TO FIRST_MAX_SCORE A NEIGHBORHOOD OF MAXIMUM SCORE POINT
			score_k[n[l]][m[l]]=first_max_score;
			for(int i=( m[l]-(int)(2./angle_resolution));i<=(m[l]+(int)(2./angle_resolution));i++)
			{
				for(int j=n[l]-(int)(20./distance_resolution);j<=n[l]+(int)(20./distance_resolution);j++)
				{
					if(j<0 || j>=width_score) continue;
					if(i<0)
					{
						d=width_score-j;
						if(0<=d && d<width_score)
							score_k[d][height_score+i]=first_max_score;
					}
					else
					{
						if(i>=height_score)
						{
							d=width_score-j;
							if(0<=d && d<width_score)
								score_k[d][i-height_score]=first_max_score;
						}
						else  score_k[j][i]=first_max_score;
					}
				}
			}
		} //END LINES LOOP
		//··········································································
		// WE SELECT THE MAXIMUN DISTORTION LEVEL ACCORDING TO THE VOTATION SCORE
		if(votation_score>max_votation_score)
		{
			max_votation_score=votation_score;

			image_primitive.set_lines(lines);
			double p = initial_distortion_parameter +
				(k*distortion_parameter_resolution);
			best_distortion_parameter= -p / (dmi + dmi*p);
			for(int l=0;l<nlineas_plus;l++){n_max[l]=n[l]; m_max[l]=m[l];}
		}
	} //END K LOOP
	//············································································
	// WE FILL THE IMAGE PRIMITIVE LENS DISTORTION MODEL
	lens_distortion_model ld;
	if(best_distortion_parameter!=0.){
		ld.set_distortion_center(point2d<double>(xc,yc));
		ld.get_d().resize(2);
		ld.get_d()[0]=1.;
		ld.get_d()[1]=best_distortion_parameter;
		image_primitive.set_distortion(ld);
	}
	else
	{
		ld.set_distortion_center(point2d<double>(xc,yc));
		ld.get_d().resize(2);
		ld.get_d()[0]=1.;
		ld.get_d()[1]=0.;
		image_primitive.set_distortion(ld);
	}
	//············································································
	// WE COMPUTE THE POINTS OF THE LINE POINTS
	// FOLLOWING THE DISTANCE OF THE POINTS TO THE LINE
	double dot_product_min=cos(ami_pi*angle_point_orientation_max_difference/180.);
	for(k=0;k<nlineas_plus;k++)
		image_primitive.get_lines()[k].get_points().clear();
	for(i=0;i<height;i++){
		for(j=0;j<width;j++){
			q=i*width+j;
			if (sp_contours_modifed.get_c()[q]==0) continue;
			x2 = sp_contours_modifed.get_x()[q];
			y2 = sp_contours_modifed.get_y()[q];
			double x2ori = subpixel_contours.get_x()[q];
			double y2ori = subpixel_contours.get_y()[q];
			point2d<double> p2(x2,y2);
			point2d<double> p2ori(x2ori,y2ori);
			point2d<double> p2d=ld.evaluation_quotient(p2);
			// ORIENTATION CORRECTION
			vector<float> corrected_orientation = orientation_update_quotient(p2,
				sp_contours_modifed.get_seno()[q],
				sp_contours_modifed.get_coseno()[q],
				image_primitive.get_distortion());
			for(k=0;k<nlineas_plus;k++){
				if(fabs(image_primitive.get_lines()[k].get_b()*corrected_orientation[0]
				-image_primitive.get_lines()[k].get_a()*corrected_orientation[1] )<
					dot_product_min) continue;
				if(fabs(image_primitive.get_lines()[k].evaluation(p2d))<
					(distance_point_line_max+1.5*distance_resolution)){
						image_primitive.get_lines()[k].get_points().push_back(p2);
						k=nlineas_plus;
				}
			}
		}
	}
	// WE RECOMPUTE THE LINE EQUATIONS
	if (lens_distortion_estimation==true) {
		for(i=0;i<(int)image_primitive.get_lines().size();i++){
			if(image_primitive.get_lines()[i].get_points().size()>2)
				distortion_points_to_line_equation_quotient(ld,image_primitive.get_lines()[i]);
		}
	}
	else{
		for(i=0;i<(int)image_primitive.get_lines().size();i++)
			if(image_primitive.get_lines()[i].get_points().size()>2)
				image_primitive.get_lines()[i].points_to_equation();
	}
	//············································································
	// WE REMOVE LINES WITH A SMALL NUMBER OF POINTS (LESS THAN 5 POINTS) AND WE 
	// JOIN LINES WHICH ARE TOO CLOSE
	dot_product_min=cos(2.*ami_pi*angle_point_orientation_max_difference/180.);
	for(k=0;k<(int)image_primitive.get_lines().size();k++){
		if(image_primitive.get_lines()[k].get_points().size()<5){
			image_primitive.get_lines().erase(image_primitive.get_lines().begin()+k);
			k--;
			continue;
		}
		for(l=k+1;l<(int)image_primitive.get_lines().size();l++){
			double paso=
				image_primitive.get_lines()[k].get_a()*image_primitive.get_lines()[l].get_a()+
				image_primitive.get_lines()[k].get_b()*image_primitive.get_lines()[l].get_b();
			if( fabs(paso)>dot_product_min){
				// WE CHECK THE AVERAGE DISTANCE OF THE POINTS TO THE LINE
				paso=0;
				for(int m=0;m<(int)image_primitive.get_lines()[l].get_points().size();m++){
					point2d<double> p2d=ld.evaluation_quotient(image_primitive.get_lines()[l].get_points()[m]);
					paso+=fabs(image_primitive.get_lines()[k].evaluation(p2d));
				}
				if( (paso/image_primitive.get_lines()[l].get_points().size())<
					3.*distance_point_line_max){
						// WE ADD THE POINTS OF THE LINE TO THE LINE POINT STRUCTURE
						image_primitive.get_lines()[k].get_points().insert(
							image_primitive.get_lines()[k].get_points().end(),
							image_primitive.get_lines()[l].get_points().begin(),
							image_primitive.get_lines()[l].get_points().end());
						// WE REMOVE THE LINE POINTS STRUCTURE
						image_primitive.get_lines().erase(image_primitive.get_lines().begin()+l);
						l--;
				}
			}
		}
	}
	// WE RECOMPUTE THE LINE EQUATIONS AFTER LINES UNION
	if (lens_distortion_estimation==true) {
		for(i=0;i<(int)image_primitive.get_lines().size();i++){
			distortion_points_to_line_equation_quotient(ld,image_primitive.get_lines()[i]);
		}
	}
	else{
		for(i=0;i<(int)image_primitive.get_lines().size();i++){
			image_primitive.get_lines()[i].points_to_equation();
		}
	}
	//············································································
	// FINALLY, WE COMPUTE AGAIN THE LINE POINTS AND DISTORTION MODEL
	// FOLLOWING THE DISTANCE OF THE POINTS TO THE LINE
	dot_product_min=cos(ami_pi*angle_point_orientation_max_difference/180.);
	for(k=0;k<(int)image_primitive.get_lines().size();k++)
		image_primitive.get_lines()[k].get_points().clear();
	for(i=0;i<height;i++){
		for(j=0;j<width;j++){
			q=i*width+j;
			if (sp_contours_modifed.get_c()[q]==0) continue;
			x2 = sp_contours_modifed.get_x()[q];
			y2 = sp_contours_modifed.get_y()[q];
			double x2ori = subpixel_contours.get_x()[q];
			double y2ori = subpixel_contours.get_y()[q];
			point2d<double> p2(x2,y2);
			point2d<double> p2ori(x2ori,y2ori);
			point2d<double> p2d=ld.evaluation_quotient(p2);
			// ORIENTATION CORRECTION
			vector<float> corrected_orientation = orientation_update_quotient(p2,
				sp_contours_modifed.get_seno()[q],
				sp_contours_modifed.get_coseno()[q],
				image_primitive.get_distortion());
			for(k=0;k<(int)image_primitive.get_lines().size();k++){
				if(fabs(image_primitive.get_lines()[k].get_b()*corrected_orientation[0]
				-image_primitive.get_lines()[k].get_a()*corrected_orientation[1] )<
					dot_product_min)
				{
					continue;
				}
				if(fabs(image_primitive.get_lines()[k].evaluation(p2d))<distance_point_line_max){
					image_primitive.get_lines()[k].get_points().push_back(p2ori);
					k=nlineas_plus;
				}
			}
		}
	}
	//············································································
	// WE FREE THE MEMORY
	free(seno); free(coseno); ami_free3d(score);
	// AND RETURN THE MAXIMUM VOTATION SCORE
	return max_votation_score;
}

