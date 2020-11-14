    #include <stdio.h>  
    #include <opencv2/opencv.hpp>  
      
    #include <vector>  
      
      
    using namespace std;  
    using namespace cv;  
      
    struct cornerInformation{  
     float x;  
     float y;  
     float x3;  
     float y3;  
     float z3;  
    };  
      
    void fprintMatrix(Mat matrix, string name);  
    void fprintfVectorMat(vector< Mat> matrix, string name);  
    void fprintf2Point( vector< vector< Point2f> > Points, string name);  
    void fprintf3Point( vector< vector< Point3f> > Points, string name);  
      
    int main(int argc, char** argv)  
    {  
     //////////////////////////////////////////////////////////////////////////////////////////////////  
     //Set input params..  
     int board_w, board_h;  
     int n_boards;  
     float measure=25;  
     Size imageSize;  
      
     vector< vector< Point2f> > imagePoints;  
     vector< vector< Point3f> > objectPoints;  
      
     board_w=6;  
     board_h=7;  
     n_boards=7;  
      
     printf("How many cross points of width direction? \n" );   
     scanf("%d", &board_w);  
     printf("How many cross points of Height direction? \n" );  
     scanf("%d", &board_h);  
      
     printf("How many board? (board will be read by this namimg-> ./pattern/p1.jpg, ./pattern/p2.jpg...\n");  
     scanf("%d", &n_boards);  
      
     printf("What mm ?\n");  
     scanf("%f", &measure);  
       
     printf("w=%d h=%d n=%d %lfmm\n", board_w, board_h, n_boards, measure);  
     //////////////////////////////////////////////////////////////////////////////////////////////////  
       
     //////////////////////////////////////////////////////////////////////////////////////////////////  
     //image load  
     //extraction image point and object point  
     char str[100];  
     for(int i=0; i< n_boards; ++i)  
     {  
      //image load  
      sprintf(str,"./pattern/p%d.jpg", i+1 );  
      printf("%s\n", str);  
      Mat img = imread(str);  
      imageSize = Size(img.cols, img.rows);  
      Mat gray;  
      cvtColor(img, gray, CV_RGB2GRAY);  
      vector< Point2f> corners;    
      
      //find chessboard corners  
      bool sCorner = findChessboardCorners(gray, Size(board_w, board_h), corners);  
      
      //if find corner success, then  
      if(sCorner)  
      {  
       //corner point refine  
       cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));  
       //draw corner  
       drawChessboardCorners(img, Size(board_w, board_h), corners, sCorner);  
       if(corners.size() == board_w*board_h)  
       {  
        vector< Point2f> v_tImgPT;  
        vector< Point3f> v_tObjPT;  
        //save 2d coordenate and world coordinate  
        for(int j=0; j< corners.size(); ++j)  
        {  
         Point2f tImgPT;  
         Point3f tObjPT;  
      
         tImgPT.x = corners[j].x;  
         tImgPT.y = corners[j].y;  
      
         tObjPT.x = j%board_w*measure;  
         tObjPT.y = j/board_w*measure;  
         tObjPT.z = 0;  
      
         v_tImgPT.push_back(tImgPT);  
         v_tObjPT.push_back(tObjPT);       
        }  
        imagePoints.push_back(v_tImgPT);  
        objectPoints.push_back(v_tObjPT);  
       }  
      }  
      sprintf(str,"Detected%d.jpg",i+1);  
      imwrite(str,img);  
      imshow("pattern",img);  
      cvWaitKey(10);  
     }  
     //////////////////////////////////////////////////////////////////////////////////////////////////  
      
      
     //////////////////////////////////////////////////////////////////////////////////////////////////  
     //calibration part  
     vector< Mat> rvecs, tvecs;  
     Mat intrinsic_Matrix(3,3, CV_64F);  
     Mat distortion_coeffs(8,1, CV_64F);  
       
     calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic_Matrix, distortion_coeffs, rvecs, tvecs);  
      
       
     for(int i=0; i< distortion_coeffs.rows; ++i)  
     {  
      for(int j=0; j< distortion_coeffs.cols; ++j)  
      {  
       printf("%lf ", distortion_coeffs.at< double>(i,j)); //cvmGet(matrix,i,j));  
      }  
      printf("\n");  
     }  
     printf("\n");  
     //////////////////////////////////////////////////////////////////////////////////////////////////  
       
      
     //////////////////////////////////////////////////////////////////////////////////////////////////  
     //save part  
     fprintMatrix(intrinsic_Matrix, "intrinsic.txt");  
     fprintMatrix(distortion_coeffs, "distortion_coeffs.txt");  
      
     fprintfVectorMat(rvecs, "rotation.txt");  
     fprintfVectorMat(tvecs, "translation.txt");  
      
     fprintf3Point(objectPoints, "objectpt.txt");  
     fprintf2Point(imagePoints, "imagept.txt");  
      
     FILE* fp=fopen("ptSize.txt","w");  
     fprintf(fp,"%d %d\n", board_w, board_h);  
     fclose(fp);  
     //////////////////////////////////////////////////////////////////////////////////////////////////  
    }  
      
    void fprintf3Point( vector< vector< Point3f> > Points, string name)  
    {  
     FILE * fp;  
     fp = fopen(name.c_str() ,"w");  
     for(int i=0; i< Points.size(); ++i)  
     {  
      for(int j=0; j< Points[i].size(); ++j)  
      {  
       fprintf(fp,"%lf %lf %lf\n", Points[i][j].x, Points[i][j].y, Points[i][j].z);  
      }  
      fprintf(fp,"\n");  
     }  
     fclose(fp);  
    }  
      
      
    void fprintf2Point( vector< vector< Point2f> > Points, string name)  
    {  
     FILE * fp;  
     fp = fopen(name.c_str() ,"w");  
     for(int i=0; i< Points.size(); ++i)  
     {  
      for(int j=0; j< Points[i].size(); ++j)  
      {  
       fprintf(fp,"%lf %lf\n", Points[i][j].x, Points[i][j].y);  
      }  
      fprintf(fp,"\n");  
     }  
     fclose(fp);  
    }  
      
      
    void fprintfVectorMat(vector< Mat> matrix, string name)  
    {  
     FILE * fp;  
     fp = fopen(name.c_str() ,"w");  
     int i,j;   
     printf("%s size %d, %d\n",name.c_str(),matrix.size(), matrix[0].cols, matrix[0].rows);  
     for(i=0; i< matrix.size(); ++i)  
     {  
      for(int j=0; j< matrix[i].rows; ++j)    
      {  
       for(int k=0; k< matrix[i].cols; ++k)  
       {  
        fprintf(fp,"%lf ", matrix[i].at<  double >(j,k));   
       }  
       fprintf(fp,"\n");  
      }  
      fprintf(fp,"\n");  
     }  
      
       
     fclose(fp);  
    }  
      
    void fprintMatrix(Mat matrix, string name)  
    {  
     FILE * fp;  
     fp = fopen(name.c_str() ,"w");  
     int i,j;   
     printf("%s size %d %d\n",name.c_str(), matrix.cols, matrix.rows);  
     for(i=0; i< matrix.rows; ++i)  
     {  
      for(j=0; j< matrix.cols; ++j)  
      {  
       fprintf(fp,"%lf ", matrix.at<  double >(i,j));   
      }  
      fprintf(fp,"\n");  
     }  
       
     fclose(fp);  
    }  
