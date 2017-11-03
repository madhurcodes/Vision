#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <cmath>
#include<vector>
using namespace cv;
using namespace std;
#include<limits>
int K = 4;
float norm(int n, float m0,float m1,float m2,float x0,float x1,float x2,float var){
    float inv_sqrt_2pi = 0.3989422804014327;
    float normConst = pow(inv_sqrt_2pi, n) * pow(var, -1.5);
    float rv = normConst * exp(-0.5 *  ((pow(m0-x0,2)+pow(m1-x1,2) + pow(m2-x2,2))/var) );
    // if((rand()/((float) numeric_limits<int>::max()))<0.00005){
    // cout<<n<<" "<<m0<<" "<<m1<<" "<<m2<<" "<<x0<<" "<<x1<<" "<<x2<<" "<<var<<endl;
    // cout<<normConst<<endl;
    // cout<<rv<<endl; 
    // }
    return rv;
}
int main(int, char**)
{
    VideoCapture cap("vtest.avi"); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
        return -1;
    }
    int K = 7;
    float T = 0.6;
    vector<vector<float> > dists(K);
    float alpha = 0.2;     
    int tt;
    for(tt=0;tt<K;tt++){
        dists[tt].push_back(0.0001);
        dists[tt].push_back(0.0001);
        dists[tt].push_back(rand()%(255 +1) + 0); 
        dists[tt].push_back(rand()%(255 +1) + 0);
        dists[tt].push_back(rand()%(255 +1) + 0);
        dists[tt].push_back(500);
        dists[tt].push_back(0);
    }
    for(int ll =0;ll<100;ll++)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        // cout<<"T is - "<< frame.type();
        Mat img2;
        frame.convertTo(img2, CV_32F); 
        // cvtColor(frame, edges, COLOR_BGR2GRAY);
        // cout<<"T- is - "<< img2.type();  
        // cout << "Width : " << img2.size().width << endl; 768
        // cout << "Height: " << img2.size().height << endl; 576

        auto comp_back = [](const std::vector<float>& a, const std::vector<float>& b) {
            return a[0] > b[0];
            };
        auto comp_prob = [](const std::vector<float>& a, const std::vector<float>& b) {
                return a[1] > b[1];
                };
        // sort(edgeList.begin(),edgeList.end(),comp);
        int tt,yy;
        vector<vector<bool> > fore_mask(576,vector<bool>(768,true));        
        Mat newimg;
        Mat fin;
        frame.convertTo(newimg, CV_32F);


        sort(dists.begin(),dists.end(),comp_back);

        float sum = 0;
        for(tt=0;tt<K;tt++){
            dists[tt][6] = 2;
        }

        for(tt=0;tt<K;tt++){
            sum = sum +  dists[tt][1];
            dists[tt][6] = 2;
            if(sum>T){
                for(yy=tt+1;yy<K;yy++){
                    dists[tt][6] = 0;
                }
                break;
            }
        }


        cout<<"Frame num - "<<ll<<endl;
        for(tt=0;tt<K;tt++){
            cout<<"Ind - "<<tt<<"  " <<dists[tt][0]<<"  " <<dists[tt][1]<<"  " <<dists[tt][2]<<"  " <<dists[tt][3]<<"  " <<dists[tt][4]<<"  " <<dists[tt][5]<<"  " <<dists[tt][6]<<endl ;  
        }

        for ( int i=0;i<576;i++) {
            for ( int j=0;j<768;j++) {
                    //each dist has rat,prob,mean0,mean1,mean2,var,isBack (0 ofr not , 2 for yes)
                    // sort(dists.begin(),dists.end(),comp_back);
                    // float sum = 0;
                    // dists[tt][6] = 2;
                    // for(tt=0;tt<K;tt++){
                    //     sum = sum +  dists[tt][0];
                    //     dists[tt][6] = 2;
                    //     if(sum>T){
                    //         for(yy=tt+1;yy<K;yy++){
                    //             dists[tt][6] = 0;
                    //         }
                    //         break;
                    //     }
                    // }
                    Vec3f bgrPixel_cv = img2.at<Vec3f>(i, j);
                    vector<float> bgrPixel;
                    bgrPixel.push_back((float)bgrPixel_cv[0]);
                    bgrPixel.push_back((float)bgrPixel_cv[1]);
                    bgrPixel.push_back((float)bgrPixel_cv[2]);
                    bool matchfound = false;
                    int matched_ind = -1;
                    sort(dists.begin(),dists.end(),comp_prob);                    
                    for(tt=0;tt<K;tt++){
                        vector<int> diffs(3);
                        diffs[0] = bgrPixel[0] - dists[tt][2];
                        diffs[1] = bgrPixel[1] - dists[tt][3];
                        diffs[2] = bgrPixel[2] - dists[tt][4];
                        float maha_dis = (diffs[0]*diffs[0]+diffs[1]*diffs[1]+diffs[2]*diffs[2])/dists[tt][5];
                        if(sqrt(maha_dis)<2.5*sqrt(dists[tt][5])) {
                            matched_ind = tt;
                            matchfound = true;
                            if(dists[tt][6]==0){
                                fore_mask[i][j] = true;//foreg
                            }
                            else if(dists[tt][6]==2){
                                fore_mask[i][j] = false;
                            }
                            else{
                                cout<<"aARGHG";
                                return 0;
                            }
                        }
                        // else{
                        // }
                    }
                    if(matchfound){
                        for(tt=0;tt<K;tt++){
                            if(tt==matched_ind){
                                dists[tt][1] = dists[tt][1]*(1-alpha) + alpha; 
                                float rho = alpha*norm(3,dists[tt][2],dists[tt][3],dists[tt][4],bgrPixel[0],bgrPixel[1],bgrPixel[2],dists[tt][5]);
                                // if((rand()/((float) numeric_limits<int>::max()))<0.000005){
                                    // cout<<dists[tt][2]<<" "<<dists[tt][3]<<" "<<dists[tt][4]<<endl;
                                    // cout<<bgrPixel[0]<<" "<<bgrPixel[1]<<" "<<bgrPixel[2]<<endl;
                                dists[tt][2] = (1- rho)*dists[tt][2] + rho*bgrPixel[0];
                                dists[tt][3] = (1- rho)*dists[tt][3] + rho*bgrPixel[1];
                                dists[tt][4] = (1- rho)*dists[tt][4] + rho*bgrPixel[2];
                                // cout<<dists[tt][2]<<" "<<dists[tt][3]<<" "<<dists[tt][4]<<endl;
                                
                                // cout<<"Rho is "<<rho<<endl;
                                // }
                                dists[tt][5] = (1-rho)*dists[tt][5] + rho*(pow(bgrPixel[0]-dists[tt][2],2)+pow(bgrPixel[1]-dists[tt][3],2)+pow(bgrPixel[2]-dists[tt][4],2) );
                            }
                            else{
                                dists[tt][1] = dists[tt][1]*(1-alpha) ;
                            }
                        }
                    }
                    else{
                        // if((rand()/((float) numeric_limits<int>::max()))<0.000005){

                        //     cout<<"REPPPPPLACC"<<endl;


                        // }
                        dists[K-1][1] = 0.0001;
                        dists[K-1][2] = bgrPixel[0];
                        dists[K-1][3] = bgrPixel[1];
                        dists[K-1][4] = bgrPixel[2];
                        dists[K-1][5] = 500;
                        // cout<<"REPPPPPLACC"<<endl;
                    }
                    for(tt=0;tt<K;tt++){
                        dists[tt][0] = dists[tt][1] / sqrt(dists[tt][5]);
                    }
                    if(fore_mask[i][j]){
                        Vec3f color;  
                        color[0]= bgrPixel[0];
                        color[1]= bgrPixel[1];
                        color[2]= bgrPixel[2];
                        newimg.at<Vec3f>(i,j) = color;
                    }
                    else{
                        Vec3f color;  
                        color[0]= 0;
                        color[1]= 0;
                        color[2]= 0;
                        newimg.at<Vec3f>(i,j) = color;
                    }
            }
        }
        newimg.convertTo(fin, CV_8U);
        imshow("Original",frame);
        imshow("Final",fin);
        // Canny(edges, edges, 0, 30, 3);
    
        // imshow("edges", edges);

        if(waitKey(5) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
