
#include "MapPoint.h"


namespace LL_SLAM
{
    long unsigned int MapPoint::nNextId=0;

    MapPoint::MapPoint(const Eigen::Vector3f &Pos){
        mWorldPos = Pos;

        mnId=nNextId++;
        nObs = 0;
        nUpdate = 0;
    }

    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos() {
        return mWorldPos;
    }

    int MapPoint::Observations()
    {
        return nObs;
    }

    std::vector<KeyFrame*> MapPoint::GetKeyFrame() {
        return mvpKF;
    }

    cv::Mat MapPoint::GetDescriptor(){
        return mDescriptor;
    }

    void MapPoint::AddObservation(KeyFrame* pKF, int cam_i, int KeyPoint_i)
    {
        mObservations.insert(pair<KeyFrame*, std::pair<int,int> >(pKF, pair<int, int>(cam_i, KeyPoint_i)));
        mvObservations.push_back(pair<KeyFrame*, std::pair<int,int> >(pKF, pair<int, int>(cam_i, KeyPoint_i)));

        bool isIn = false;
        for (int i = 0 ; i < mvpKF.size(); i++) {
            if (mvpKF[i] == pKF) {
                isIn = true;
            }
        }
        if (isIn == false) {mvpKF.push_back(pKF);}

        nObs++;

//
//        //mDescriptor
//        std::multimap<KeyFrame*,std::pair<int,int> > observations = mObservations ;
//        vector<cv::Mat> vDescriptors;
//
//        vDescriptors.reserve(observations.size());
//        for(multimap<KeyFrame*,pair<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
//        {
//            KeyFrame* pKFForDesc = mit->first;
//            int cam_iForDesc = mit->second.first; int KeyPoint_iForDesc = mit -> second.second;
//            vDescriptors.push_back(pKFForDesc->mvCamDescriptors[cam_iForDesc].row(KeyPoint_iForDesc));
//        }
//
//
//        if(vDescriptors.empty())
//            return;
//
//        if (vDescriptors.size() > mDistances.size()) {
//            if (mDistances.size() == 0) {
//               mDistances = vector<vector<float>>(50, vector<float>(50, -1));
//            } else {
//                vector<vector<float>> Distances(vDescriptors.size()*2, vector<float>(vDescriptors.size()*2, -1));
//                for (int i = 0; i < mDistances.size(); i++) {
//                    for (int j = 0; j < mDistances[i].size(); j++) {
//                        Distances[i][j] = mDistances[i][j];
//                    }
//                }
//                mDistances = Distances;
//            }
//        }
//
//        // Compute distances between them
//        const size_t N = vDescriptors.size();
//
////        float Distances[N][N];
////        for(size_t i=0;i<N;i++)
////        {
////            Distances[i][i]=0;
////            for(size_t j=i+1;j<N;j++)
////            {
////                int distij = FeatureMatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
////                Distances[i][j]=distij;
////                Distances[j][i]=distij;
////            }
////        }
//
//        for(size_t i=0;i<N;i++)
//        {
//            mDistances[i][i]=0;
//            for(size_t j=i+1;j<N;j++)
//            {
//                if (mDistances[i][j] >= 0) {continue;}
//                int distij = FeatureMatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
//                mDistances[i][j]=distij;
//                mDistances[j][i]=distij;
//            }
//        }
//
//        // Take the descriptor with least median distance to the rest
//        int BestMedian = INT_MAX;
//        int BestIdx = 0;
//        for(size_t i=0;i<N;i++)
//        {
////            vector<int> vDists(Distances[i],Distances[i]+N);
//            vector<int> vDists;
//            for (int j = 0; j < mDistances[i].size(); j++) {
//                if (mDistances[i][j] == -1) {continue;}
//                vDists.push_back(mDistances[i][j]);
//            }
//
//            sort(vDists.begin(),vDists.end());
//            int median = vDists[0.5*(N-1)];
//
//            if(median<BestMedian)
//            {
//                BestMedian = median;
//                BestIdx = i;
//            }
//        }
//
//        mDescriptor = vDescriptors[BestIdx].clone();

//        mDescriptor = pKF->mvCamDescriptors[cam_i].row(KeyPoint_i).clone();

        UpdateDescriptor();

        mColor = pKF->mvColor[cam_i][KeyPoint_i];
    }


    void MapPoint::UpdateDescriptor() {

        if (!mDescriptor.empty() && nObs - nUpdate < 10) {
            return;
        }
        nUpdate = nObs;

        //mDescriptor
        std::vector<pair<KeyFrame*,std::pair<int,int>> >  observations = mvObservations ;
        vector<cv::Mat> vDescriptors;

        vDescriptors.reserve(observations.size());
        for(vector<pair<KeyFrame*,std::pair<int,int>> > ::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFForDesc = mit->first;
            int cam_iForDesc = mit->second.first; int KeyPoint_iForDesc = mit -> second.second;
            vDescriptors.push_back(pKFForDesc->mvCamDescriptors[cam_iForDesc].row(KeyPoint_iForDesc));
        }


        if(vDescriptors.empty())
            return;

        if (vDescriptors.size() > mDistances.size()) {
            if (mDistances.size() == 0) {
               mDistances = vector<vector<float>>(50, vector<float>(50, -1));
            } else {
                vector<vector<float>> Distances(vDescriptors.size()*2, vector<float>(vDescriptors.size()*2, -1));
                for (int i = 0; i < mDistances.size(); i++) {
                    for (int j = 0; j < mDistances[i].size(); j++) {
                        Distances[i][j] = mDistances[i][j];
                    }
                }
                mDistances = Distances;
                //swap(mDistances, Distances);
                if (vDescriptors.size() > mDistances.size()) {
                    cout << "vDescriptors error 1" << endl;
                }
            }
        }

        if (vDescriptors.size() > mDistances.size()) {
            cout << "vDescriptors error 2" << endl;
        }
        // Compute distances between them
        const size_t N = vDescriptors.size();

        for(size_t i=0;i<N;i++)
        {
            mDistances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                if (mDistances[i][j] >= 0) {continue;}
                int distij = FeatureMatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
                mDistances[i][j]=distij;
                mDistances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists;
            for (int j = 0; j < mDistances[i].size(); j++) {
                if (mDistances[i][j] == -1) {continue;}
                vDists.push_back(mDistances[i][j]);
            }

            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        mDescriptor = vDescriptors[BestIdx].clone();
    }



} //namespace ORB_SLAM
