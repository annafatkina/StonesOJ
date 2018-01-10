/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */


#include "utilities.h"
#include "../ami_primitives/point2d.h"

//------------------------------------------------------------------------------

/**
 * \fn std::vector < std::vector <unsigned int> >
        ami::utilities::boundary_neighborhood_9n(const unsigned int width,
                                                 const unsigned int height)
 * \brief function to compute the 9 neighborhood index of the image boundary
 * \author Luis Alvarez
 * \return b[k][l] where for a boundary point k : b[k][0]=m (image index point),
          b[k][1]=m-width, b[k][2]=m+width, b[k][3]=m+1, b[k][4]=m-1,
          b[k][5]=m-width+1, b[k][6]=m+width+1, b[k][7]=m-width-1,
          b[k][8]=m+width-1 (if point is outside the image we take the nearest
                             one inside the image)
*/

std::vector < std::vector <unsigned int> >
  boundary_neighborhood_9n(const unsigned int width, const unsigned int height)
{
  if( width<3 || height<3) return(std::vector< std::vector <unsigned int> >());
  unsigned int size=2*(width+height)-4;
  std::vector< std::vector <unsigned int> > neigh_vector(size,
                                                  std::vector<unsigned int>(9));

  unsigned int m,cont=0;

  // FIRST CORNER
  neigh_vector[cont][0]=0; // pixel position index
  neigh_vector[cont][1]=0; // North pixel position index
  neigh_vector[cont][2]=width; // South pixel position index
  neigh_vector[cont][3]=1; // Est pixel position index
  neigh_vector[cont][4]=0; // West pixel position index
  neigh_vector[cont][5]=1; // North-Est pixel position index
  neigh_vector[cont][6]=width+1; // South-Est pixel position index
  neigh_vector[cont][7]=0; // North-West pixel position index
  neigh_vector[cont][8]=0; // South-West pixel position index
  cont++;

  // BOUNDARY HORIZONTAL LINE
  for(unsigned int k=1,k_end=width-1;k!=k_end;++k,++cont){
    neigh_vector[cont][0]=k; // pixel position index
    neigh_vector[cont][1]=k; // North pixel position index
    neigh_vector[cont][2]=k+width; // South pixel position index
    neigh_vector[cont][3]=k+1; // Est pixel position index
    neigh_vector[cont][4]=k-1; // West pixel position index
    neigh_vector[cont][5]=k+1; // North-Est pixel position index
    neigh_vector[cont][6]=k+width+1; // South-Est pixel position index
    neigh_vector[cont][7]=k-1; // North-West pixel position index
    neigh_vector[cont][8]=k+width-1; // South-West pixel position index
  }

  // SECOND CORNER
  m=width-1;
  neigh_vector[cont][0]=m; // pixel position index
  neigh_vector[cont][1]=m; // North pixel position index
  neigh_vector[cont][2]=m+width; // South pixel position index
  neigh_vector[cont][3]=m; // Est pixel position index
  neigh_vector[cont][4]=m-1; // West pixel position index
  neigh_vector[cont][5]=m; // North-Est pixel position index
  neigh_vector[cont][6]=m+width; // South-Est pixel position index
  neigh_vector[cont][7]=m-1; // North-West pixel position index
  neigh_vector[cont][8]=m+width-1; // South-West pixel position index
  cont++;

  // BOUNDARY VERTICAL LINE
  for(unsigned int k=1,k_end=height-1;k!=k_end;++k,++cont){
    m=(k+1)*width-1;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m+width; // South pixel position index
    neigh_vector[cont][3]=m; // Est pixel position index
    neigh_vector[cont][4]=m-1; // West pixel position index
    neigh_vector[cont][5]=m-width; // North-Est pixel position index
    neigh_vector[cont][6]=m+width; // South-Est pixel position index
    neigh_vector[cont][7]=m-width-1; // North-West pixel position index
    neigh_vector[cont][8]=m+width-1; // South-West pixel position index

  }

  // CORNER POINT
  m=height*width-1;
  neigh_vector[cont][0]=m; // pixel position index
  neigh_vector[cont][1]=m-width; // North pixel position index
  neigh_vector[cont][2]=m; // South pixel position index
  neigh_vector[cont][3]=m; // Est pixel position index
  neigh_vector[cont][4]=m-1; // West pixel position index
  neigh_vector[cont][5]=m-width; // North-Est pixel position index
  neigh_vector[cont][6]=m; // South-Est pixel position index
  neigh_vector[cont][7]=m-width-1; // North-West pixel position index
  neigh_vector[cont][8]=m-1; // South-West pixel position index
  cont++;

  // HORIZONTAL LINE
  for(unsigned int k=2,k_end=width;k!=k_end;++k,++cont){
    m=height*width-k;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m; // South pixel position index
    neigh_vector[cont][3]=m+1; // Est pixel position index
    neigh_vector[cont][4]=m-1; // West pixel position index
    neigh_vector[cont][5]=m-width+1; // North-Est pixel position index
    neigh_vector[cont][6]=m+1; // South-Est pixel position index
    neigh_vector[cont][7]=m-width-1; // North-West pixel position index
    neigh_vector[cont][8]=m-1; // South-West pixel position index
  }

  // CORNER POINT
  m=(height-1)*width;
  neigh_vector[cont][0]=m; // pixel position index
  neigh_vector[cont][1]=m-width; // North pixel position index
  neigh_vector[cont][2]=m; // South pixel position index
  neigh_vector[cont][3]=m+1; // Est pixel position index
  neigh_vector[cont][4]=m; // West pixel position index
  neigh_vector[cont][5]=m-width+1; // North-Est pixel position index
  neigh_vector[cont][6]=m+1; // South-Est pixel position index
  neigh_vector[cont][7]=m-width; // North-West pixel position index
  neigh_vector[cont][8]=m; // South-West pixel position index
  cont++;

  // VERTICAL LINE
  for(unsigned int k=2,k_end=height;k!=k_end;++k,++cont){
    m=(height-k)*width;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m+width; // South pixel position index
    neigh_vector[cont][3]=m+1; // Est pixel position index
    neigh_vector[cont][4]=m; // West pixel position index
    neigh_vector[cont][5]=m-width+1; // North-Est pixel position index
    neigh_vector[cont][6]=m+width+1; // South-Est pixel position index
    neigh_vector[cont][7]=m-width; // North-West pixel position index
    neigh_vector[cont][8]=m+width; // South-West pixel position index
  }

  return(neigh_vector);
}

//------------------------------------------------------------------------------

/**
 * \fn std::vector < std::vector <unsigned int> >
        ami::utilities::boundary_neighborhood_5n(const unsigned int width,
                                                 const unsigned int height)
 * \brief function to compute the 5 neighborhood index of the image boundary
 * \author Luis Alvarez
 * \return b[k][l] where for a boundary point k : b[k][0]=m (image index point),
            b[k][1]=m-width, b[k][2]=m+width, b[k][3]=m+1, b[k][4]=m-1,
            (if point is outside the image we take the nearest one inside the
             image)
*/
std::vector < std::vector <unsigned int> >
  boundary_neighborhood_5n(const unsigned int width, const unsigned int height)
{
  if( width<3 || height<3) return(std::vector< std::vector <unsigned int> >());
  unsigned int size=2*(width+height)-4;
  std::vector< std::vector <unsigned int> > neigh_vector(size,
                                                  std::vector<unsigned int>(5));

  unsigned int m,cont=0;

  // FIRST CORNER
  neigh_vector[cont][0]=0; // pixel position index
  neigh_vector[cont][1]=0; // North pixel position index
  neigh_vector[cont][2]=width; // South pixel position index
  neigh_vector[cont][3]=1; // Est pixel position index
  neigh_vector[cont][4]=0; // West pixel position index
  cont++;

  // BOUNDARY HORIZONTAL LINE
  for(unsigned int k=1,k_end=width-1;k!=k_end;++k,++cont){
    neigh_vector[cont][0]=k; // pixel position index
    neigh_vector[cont][1]=k; // North pixel position index
    neigh_vector[cont][2]=k+width; // South pixel position index
    neigh_vector[cont][3]=k+1; // Est pixel position index
    neigh_vector[cont][4]=k-1; // West pixel position index
  }

  // SECOND CORNER
  neigh_vector[cont][0]=width-1; // pixel position index
  neigh_vector[cont][1]=width-1; // North pixel position index
  neigh_vector[cont][2]=2*width-1; // South pixel position index
  neigh_vector[cont][3]=width-1; // Est pixel position index
  neigh_vector[cont][4]=width-2; // West pixel position index
  cont++;

  // BOUNDARY VERTICAL LINE
  for(unsigned int k=1,k_end=height-1;k!=k_end;++k,++cont){
    m=(k+1)*width-1;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m+width; // South pixel position index
    neigh_vector[cont][3]=m; // Est pixel position index
    neigh_vector[cont][4]=m-1; // West pixel position index
  }

  // CORNER POINT
  m=height*width-1;
  neigh_vector[cont][0]=m; // pixel position index
  neigh_vector[cont][1]=m-width; // North pixel position index
  neigh_vector[cont][2]=m; // South pixel position index
  neigh_vector[cont][3]=m; // Est pixel position index
  neigh_vector[cont][4]=m-1; // West pixel position index
  cont++;

  // HORIZONTAL LINE
  for(unsigned int k=2,k_end=width;k!=k_end;++k,++cont){
    m=height*width-k;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m; // South pixel position index
    neigh_vector[cont][3]=m+1; // Est pixel position index
    neigh_vector[cont][4]=m-1; // West pixel position index
  }

  // CORNER POINT
  m=(height-1)*width;
  neigh_vector[cont][0]=m; // pixel position index
  neigh_vector[cont][1]=m-width; // North pixel position index
  neigh_vector[cont][2]=m; // South pixel position index
  neigh_vector[cont][3]=m+1; // Est pixel position index
  neigh_vector[cont][4]=m; // West pixel position index
  cont++;

  // VERTICAL LINE
  for(unsigned int k=2,k_end=height;k!=k_end;++k,++cont){
    m=(height-k)*width;
    neigh_vector[cont][0]=m; // pixel position index
    neigh_vector[cont][1]=m-width; // North pixel position index
    neigh_vector[cont][2]=m+width; // South pixel position index
    neigh_vector[cont][3]=m+1; // Est pixel position index
    neigh_vector[cont][4]=m; // West pixel position index
  }

  return(neigh_vector);
}

//------------------------------------------------------------------------------

/* FUNCTION TO CONVERT FROM HSV COLOR SPACE TO RGB */
void ami_hsv2rgb(float h,float s,float v,
                    unsigned char *r,unsigned char *g,unsigned char *b)
{
    float
    H = h*360./255.,
    S = s/255.,
    V = v/255.,
    R = 0, G = 0, B = 0;
    if (H==0 && S==0) R = G = B = V;
    else
    {
        H/=60.;
        const int i = (int)floor((int) H)%6;
        const float
          f = (i&1)?(H - i):(1 - H + i),
          m = V*(1 - S),
          n = V*(1 - S*f);
        switch (i)
        {
            case 6 :
            case 0 : R = V; G = n; B = m; break;
            case 1 : R = n; G = V; B = m; break;
            case 2 : R = m; G = V; B = n; break;
            case 3 : R = m; G = n; B = V; break;
            case 4 : R = n; G = m; B = V; break;
            case 5 : R = V; G = m; B = n; break;
        }
    }
    R*=255; G*=255; B*=255;
    *r = (unsigned char) (R<0?0:(R>255?255:R));
    *g = (unsigned char) (G<0?0:(G>255?255:G));
    *b = (unsigned char) (B<0?0:(B>255?255:B));
}

//------------------------------------------------------------------------------

void drawHoughLines(image_primitives ip, ami::image<unsigned char> &bn)
{
	//DRAW THE LINES IN THE IMAGE
	ami::image_draw imgd;

	for(int i = 0; i<(int)ip.get_lines().size(); i++)
	{
		//We obtain the color for the points associated with a line
		float h[25]={0.,80.,160.,240.,40.,120.,200.,20.,100.,180.,60.,140.,220.,
								 10.,90.,170.,30.,110.,190.,50.,130.,210.,70.,150.,230.};
		unsigned char r,g,b;
		ami_hsv2rgb(h[i%25] ,255.,255.,&r,&g,&b);
		for(int j = 0; j<(int)ip.get_lines()[i].get_points().size(); j++)
		{
			point2d<double> ori = ip.get_lines()[i].get_points()[j];
			//DRAW THE ORIGINAL POINT IN r,g,b
			imgd.draw_cercle(bn,ori.x,ori.y,(float)4.,r,g,b);
		}
	}
}

//------------------------------------------------------------------------------

void invert(ami::image<unsigned char> &input,
            ami::image<unsigned char> &output)
{
	int tam = input.width()*input.height()*input.nChannels();
	for(int i=0; i<tam; i++) output[i] = 255 - input[i];
}

//------------------------------------------------------------------------------

void print_function_syntax_lens_distortion_correction_division_model_1p()
{
	cout << "FUNCTION SYNTAX:" << endl;
	cout << "exe_file  input.png  output_canny.png  output_hough.png" << endl;
	cout << "          output_corrected_image.png high_treshold_Canny" << endl;
	cout << "          initial_distortion_parameter_hough" << endl;
	cout << "          final_distortion_parameter_hough" << endl;
	cout << "          distance_point_line_max_hough" << endl;
	cout << "          angle_point_orientation_max_difference" << endl;
	cout << "	         primitives_file" << endl;
	cout << "Parameters description:" << endl;
	cout << "  exe_file: executable name, by default:" << endl;
	cout << "            lens_distortion_correction_division_model_1p"
			 << endl;
	cout << "  inpug.png: input image." << endl;
	cout << "  output_canny.png: output image with the detected edges by means"
			 << endl;
	cout << "                    of Canny method." << endl;
	cout << "  output_hough.png: output image with the detected lines by means"
			 << endl;
	cout << "                    of improved Hough transform." << endl;
	cout << "  output_corrected_image.png: output image with the corrected"
			 << endl;
	cout << "                              distortion." << endl;
	cout << "  high_threshold_Canny: float value for the high threshold of the"
			 << endl;
	cout << "                        Canny method (between 0.7 and 1.0)"
			 << endl;
	cout << "  initial_distortion_parameter: float value for the initial"
	     << endl;
	cout << "                                normalized distortion parameter"
			 << endl;
	cout << "                                (greater or equal to -0.5)" << endl;
	cout << "  final_distortion_parameter: float value for the final normalized"
			 << endl;
	cout << "                              distortion parameter (greater or equal"
			 << endl;
	cout << "                              to the initial value)" << endl;
	cout << "  distance_point_line_max_hough: maximum distance allowed between"
			 << endl;
	cout << "                                 points and associated lines."
			 << endl;
	cout << "  angle_point_orientation_max_difference: maximum difference" << endl;
	cout << "                                 (in degrees) of the point" << endl;
	cout << "                                 orientation angle and the" << endl;
	cout << "                                 line angle." << endl;
  cout << "  primitives_file: file for saving information about the detected"<<endl;
	cout << "                   primitives." <<endl << endl;
	cout << "Example command:" << endl;
	cout << " ./lens_distortion_correction_division_model_1p example/pattern.png" << endl;
	cout << " pattern_canny.png pattern_hough.png pattern_corrected_image.png 0.8"
       << endl;
	cout << " 0.0 1.0 3.0 2.0 primitives.txt" << endl;
}

//------------------------------------------------------------------------------

int check_params_lens_distortion_correction_division_model_1p(char *argv[])
{
	//Messages
	string error_message_head("The argument ");
	string error_message_image(" must be a png image.\n");
	string error_message_numgt(" must be greater than ");
	string error_message_numgte(" must be greater or equal to ");
	string error_message("");

	//Input image
	string str(argv[1]);
	int pos = str.find_last_of('.');
	string ext = str.substr(pos+1);
	if(ext != "png" && ext != "PNG")
		error_message += error_message_head + "1 (input image)" +
										 error_message_image;

	//Canny image
	str = string(argv[2]);
	pos = str.find_last_of('.');
	ext = str.substr(pos+1);
	if(ext != "png" && ext != "PNG")
		error_message += error_message_head + "2 (output Canny image)" +
										 error_message_image;

	//Hough image
	str = string(argv[3]);
	pos = str.find_last_of('.');
	ext = str.substr(pos+1);
	if(ext != "png" && ext != "PNG")
		error_message += error_message_head + "3 (output Hough lines image)" +
										 error_message_image;

	//Corrected image
	str = string(argv[4]);
	pos = str.find_last_of('.');
	ext = str.substr(pos+1);
	if(ext != "png" && ext != "PNG")
		error_message += error_message_head + "4 (output corrected image)" +
										 error_message_image;

	//High threshold Canny
	float num = atof(argv[5]);
	if(num <= 0.7)
		error_message += error_message_head + "5 (high threshold Canny)" +
										 error_message_numgt + "0.7\n";

	//Initial distortion parameter
	num = atof(argv[6]);
	if(num<-0.5)
		error_message += error_message_head + "6 (initial distortion parameter)" +
										 error_message_numgte + "-0.5\n";

	//Final distortion parameter
	num = atof(argv[7]);
	if(num < atof(argv[6]))
		error_message += error_message_head + "7 (final distortion parameter)" +
										 error_message_numgte + argv[6]+"\n";

	//Maximum distance between points and line
	num = atof(argv[8]);
	if(num < 0.0)
		error_message += error_message_head +
										 "8 (maximum distance between points and line)" +
										 error_message_numgte + "0.0\n";

	//Maximum difference between point angle and line angle
	num = atof(argv[9]);
	if(num < 0.0 || num > 45.0)
		error_message += error_message_head + "9 (maximum difference between point"+
									" angle and line angle) must be between 0.0 and 45.0\n";

	if(error_message.length() > 0)
	{
		cout << error_message;
		return -1;
	}
	return 0;
}

//------------------------------------------------------------------------------

int count_points(image_primitives ip)
{
	int count = 0;
	for(int i=0; i<(int)ip.get_lines().size(); i++)
		count += ip.get_lines()[i].get_points().size();
	return count;
}

//------------------------------------------------------------------------------

void manage_failure(char *argv[])
{
	ami::image<unsigned char> input(argv[1]);
	//Write the output images as a copy of the input
	input.write(argv[2]);
	input.write(argv[3]);
	input.write(argv[4]);
	//Write output file
	ofstream fs("output.txt");
	fs << "Selected parameters:" << endl;
    fs << "\t High Canny's threshold: " << argv[5] << endl;
    fs << "\t Initial normalized distortion parameter: " << argv[6] << endl;
    fs << "\t Final normalized distortion parameter: " << argv[7] << endl;
    fs << "\t Maximum distance between points and line: " << argv[8] << endl;
    fs << "\t Maximum differente between edge point and line orientations: "
     << argv[9]	<< endl;
    fs << "-------------------------" << endl;
    fs << "Results: " << endl;
	fs << "\t Program failed. Probably due to a bad parameter choice " << endl;
	fs.close();
	//And write an empty output file of primitives information
	ofstream pf(argv[10]);
	pf.close();
}
