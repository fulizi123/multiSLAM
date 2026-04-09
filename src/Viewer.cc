
#include "Viewer.h"
#include "MapObject.h"

#include <array>
#include <cerrno>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_set>

namespace
{
    bool EnsureDirectory(const std::string &path)
    {
        if (path.empty()) {
            return false;
        }
        if (mkdir(path.c_str(), 0777) == 0) {
            return true;
        }
        return errno == EEXIST;
    }

    cv::Mat MakePreviewImage(const cv::Mat &image, const cv::Size &targetSize)
    {
        cv::Mat preview;
        if (image.channels() == 4) {
            cv::cvtColor(image, preview, cv::COLOR_BGRA2BGR);
        } else if (image.channels() == 1) {
            cv::cvtColor(image, preview, cv::COLOR_GRAY2BGR);
        } else {
            preview = image.clone();
        }

        if (targetSize.width > 0 && targetSize.height > 0 && preview.size() != targetSize) {
            cv::resize(preview, preview, targetSize);
        }
        return preview;
    }

    void CopyPreviewToCanvas(const cv::Mat &preview, cv::Mat &canvas, int x, int y)
    {
        int dstX = x < 0 ? 0 : x;
        int dstY = y < 0 ? 0 : y;
        int copyWidth = preview.cols;
        int copyHeight = preview.rows;

        if (dstX >= canvas.cols || dstY >= canvas.rows) {
            return;
        }

        if (dstX + copyWidth > canvas.cols) {
            copyWidth = canvas.cols - dstX;
        }
        if (dstY + copyHeight > canvas.rows) {
            copyHeight = canvas.rows - dstY;
        }
        if (copyWidth <= 0 || copyHeight <= 0) {
            return;
        }

        preview(cv::Rect(0, 0, copyWidth, copyHeight)).copyTo(canvas(cv::Rect(dstX, dstY, copyWidth, copyHeight)));
    }

    cv::Size GetSurroundPreviewSize(const cv::Mat &image, int canvasWidth, int canvasHeight)
    {
        int previewWidth = canvasWidth / 4;
        if (previewWidth < 160) {
            previewWidth = 160;
        }

        int previewHeight = previewWidth;
        if (image.cols > 0 && image.rows > 0) {
            previewHeight = previewWidth * image.rows / image.cols;
        }

        int maxPreviewHeight = canvasHeight / 5;
        if (maxPreviewHeight < 100) {
            maxPreviewHeight = 100;
        }

        if (previewHeight > maxPreviewHeight && previewHeight > 0) {
            previewWidth = previewWidth * maxPreviewHeight / previewHeight;
            previewHeight = maxPreviewHeight;
        }

        return cv::Size(previewWidth, previewHeight);
    }

    cv::Point2f ScalePointToPreview(const cv::Point2f &point, const cv::Size &sourceSize, const cv::Size &previewSize)
    {
        if (sourceSize.width <= 0 || sourceSize.height <= 0 || previewSize.width <= 0 || previewSize.height <= 0) {
            return point;
        }

        float scaleX = static_cast<float>(previewSize.width) / static_cast<float>(sourceSize.width);
        float scaleY = static_cast<float>(previewSize.height) / static_cast<float>(sourceSize.height);
        return cv::Point2f(point.x * scaleX, point.y * scaleY);
    }

    LL_SLAM::MapObject::ObjectState ResolveVisualizationState(const LL_SLAM::ObjectObservation &obj,
                                                              const LL_SLAM::MapObject *pMapObject)
    {
        if (pMapObject != nullptr && !pMapObject->isBad()) {
            return pMapObject->GetState();
        }
        return obj.is_static ? LL_SLAM::MapObject::OBJECT_STATE_STATIC
                             : LL_SLAM::MapObject::OBJECT_STATE_DYNAMIC;
    }

    LL_SLAM::MapObject *ResolveTrackedMapObject(const LL_SLAM::Frame *pFrame, int objectIdx)
    {
        if (pFrame == nullptr || objectIdx < 0 || objectIdx >= int(pFrame->mvObjectObservations.size())) {
            return nullptr;
        }

        LL_SLAM::MapObject *pMapObject =
            objectIdx < int(pFrame->mvMapObjects.size()) ? pFrame->mvMapObjects[objectIdx] : nullptr;
        if (pMapObject != nullptr && !pMapObject->isBad()) {
            return pMapObject;
        }

        if (pFrame->mpSystem == nullptr || pFrame->mpSystem->mpMap == nullptr) {
            return nullptr;
        }

        const LL_SLAM::ObjectObservation &obj = pFrame->mvObjectObservations[objectIdx];
        if (obj.track_id < 0) {
            return nullptr;
        }
        return pFrame->mpSystem->mpMap->GetMapObjectByTrackId(obj.track_id);
    }

    cv::Scalar GetObjectColor(LL_SLAM::MapObject::ObjectState state, bool emphasize)
    {
        if (state == LL_SLAM::MapObject::OBJECT_STATE_STATIC) {
            return emphasize ? cv::Scalar(70, 235, 110) : cv::Scalar(110, 255, 160);
        }
        if (state == LL_SLAM::MapObject::OBJECT_STATE_UNKNOWN) {
            return emphasize ? cv::Scalar(255, 110, 255) : cv::Scalar(235, 150, 255);
        }
        return emphasize ? cv::Scalar(40, 180, 255) : cv::Scalar(30, 110, 200);
    }
    cv::Scalar GetTopologySemanticColor(const LL_SLAM::CarlaTopologyStatus &status)
    {
        if (status.is_junction) {
            return cv::Scalar(170, 220, 255);
        }

        const unsigned int seed = static_cast<unsigned int>(status.road_id) * 2654435761u;
        const int blue = 170 + int(seed & 0x1F);
        const int green = 175 + int((seed >> 5) & 0x1F);
        const int red = 180 + int((seed >> 10) & 0x1F);
        return cv::Scalar(blue, green, red);
    }

    std::vector<cv::Point> BuildTopologySemanticSegmentPolygon(
        const Eigen::Matrix4f &TwbPrev,
        const Eigen::Matrix4f &TwbCurr,
        const Eigen::Vector3f &viewerPos,
        float fbl,
        const cv::Size &canvasSize)
    {
        std::vector<cv::Point> polygon;
        if (canvasSize.width <= 0 || canvasSize.height <= 0 || fbl <= 1e-6f) {
            return polygon;
        }

        const float halfWidth = 7.0f;
        const Eigen::Matrix3f RwbPrev = LL_SLAM::CommonTools::T2R(TwbPrev);
        const Eigen::Matrix3f RwbCurr = LL_SLAM::CommonTools::T2R(TwbCurr);
        const Eigen::Vector3f twbPrev = LL_SLAM::CommonTools::T2t(TwbPrev);
        const Eigen::Vector3f twbCurr = LL_SLAM::CommonTools::T2t(TwbCurr);

        const Eigen::Vector3f prevLeft = RwbPrev * Eigen::Vector3f(-halfWidth, 0.0f, 0.0f) + twbPrev - viewerPos;
        const Eigen::Vector3f prevRight = RwbPrev * Eigen::Vector3f(halfWidth, 0.0f, 0.0f) + twbPrev - viewerPos;
        const Eigen::Vector3f currLeft = RwbCurr * Eigen::Vector3f(-halfWidth, 0.0f, 0.0f) + twbCurr - viewerPos;
        const Eigen::Vector3f currRight = RwbCurr * Eigen::Vector3f(halfWidth, 0.0f, 0.0f) + twbCurr - viewerPos;

        auto toCanvas = [&](const Eigen::Vector3f &p) -> cv::Point {
            return cv::Point(
                int(p.x() / fbl + canvasSize.width * 0.5f),
                int(-p.z() / fbl + canvasSize.height * 0.5f)
            );
        };

        polygon.reserve(4);
        polygon.push_back(toCanvas(prevLeft));
        polygon.push_back(toCanvas(prevRight));
        polygon.push_back(toCanvas(currRight));
        polygon.push_back(toCanvas(currLeft));
        return polygon;
    }

    void DrawTopologySemanticBackground(cv::Mat &canvas,
                                        const std::vector<Eigen::Matrix4f> &twbs,
                                        const std::vector<LL_SLAM::CarlaTopologyStatus> &statusHistory,
                                        const Eigen::Vector3f &viewerPos,
                                        float fbl)
    {
        if (canvas.empty() || twbs.size() < 2 || statusHistory.size() < 2) {
            return;
        }

        const size_t n = std::min(twbs.size(), statusHistory.size());
        cv::Mat overlay = canvas.clone();
        bool hasSemanticRibbon = false;
        for (size_t i = 1; i < n; ++i) {
            const LL_SLAM::CarlaTopologyStatus &status = statusHistory[i];
            if (!status.valid) {
                continue;
            }
            const std::vector<cv::Point> polygon = BuildTopologySemanticSegmentPolygon(
                twbs[i - 1], twbs[i], viewerPos, fbl, canvas.size());
            if (polygon.size() < 4) {
                continue;
            }
            cv::fillConvexPoly(overlay, polygon, GetTopologySemanticColor(status), cv::LINE_AA);
            hasSemanticRibbon = true;
        }

        if (hasSemanticRibbon) {
            cv::addWeighted(overlay, 0.30, canvas, 0.70, 0.0, canvas);
        }
    }

    void DrawTopologySemanticLegend(cv::Mat &canvas, const LL_SLAM::Frame *pFrame)
    {
        if (canvas.empty() || pFrame == nullptr || !pFrame->mCarlaTopologyStatus.valid) {
            return;
        }

        const LL_SLAM::CarlaTopologyStatus &status = pFrame->mCarlaTopologyStatus;
        std::ostringstream oss;
        oss << "topo : " << (status.is_junction ? "junction" : "road")
            << " road : " << status.road_id
            << " lane : " << status.lane_id
            << " sec : " << status.section_id;
        if (status.is_junction) {
            oss << " jid : " << status.junction_id;
        } else if (status.next_junction_distance_m >= 0.0f) {
            oss << std::fixed << std::setprecision(1)
                << " jdist : " << status.next_junction_distance_m;
        }

        cv::putText(canvas, oss.str(), cv::Point(12, 44), cv::FONT_HERSHEY_PLAIN, 1.15,
                    cv::Scalar(215, 215, 215), 1, cv::LINE_AA);
    }


    std::vector<int> SelectSurroundPreviewCameras(const LL_SLAM::Frame *pFrame)
    {
        if (pFrame == nullptr || pFrame->mNumCam <= 0) {
            return {};
        }
        // Fixed 12-camera rig layout from Carla.yaml:
        // top-left/front-left, top-center/front, top-right/front-right,
        // right, rear-right, rear, rear-left, left.
        const std::vector<int> preferredOrder = {9, 1, 5, 4, 3, 7, 11, 10};

        std::vector<int> selected;
        selected.reserve(std::min<int>(preferredOrder.size(), pFrame->mNumCam));
        for (int camIndex : preferredOrder) {
            if (camIndex >= 0 && camIndex < pFrame->mNumCam) {
                selected.push_back(camIndex);
            }
        }

        if (selected.empty()) {
            for (int camIndex = 0; camIndex < pFrame->mNumCam; ++camIndex) {
                selected.push_back(camIndex);
            }
        }
        return selected;
    }

    std::array<Eigen::Vector3f, 8> BuildCuboidCorners(const Eigen::Vector3f &center,
                                                      const Eigen::Quaternionf &rotation,
                                                      const Eigen::Vector3f &size)
    {
        // ObjectTrackTruth stores box size as [width, length, height], while the
        // box pose quaternion uses the original nuScenes object frame:
        // local x: forward, local y: left, local z: up.
        const float halfWidth = 0.5f * size.x();
        const float halfLength = 0.5f * size.y();
        const float halfHeight = 0.5f * size.z();
        const std::array<Eigen::Vector3f, 8> localCorners = {{
            Eigen::Vector3f(-halfLength, -halfWidth, -halfHeight),
            Eigen::Vector3f( halfLength, -halfWidth, -halfHeight),
            Eigen::Vector3f( halfLength,  halfWidth, -halfHeight),
            Eigen::Vector3f(-halfLength,  halfWidth, -halfHeight),
            Eigen::Vector3f(-halfLength, -halfWidth,  halfHeight),
            Eigen::Vector3f( halfLength, -halfWidth,  halfHeight),
            Eigen::Vector3f( halfLength,  halfWidth,  halfHeight),
            Eigen::Vector3f(-halfLength,  halfWidth,  halfHeight)
        }};

        std::array<Eigen::Vector3f, 8> worldCorners;
        const Eigen::Matrix3f R = rotation.normalized().toRotationMatrix();
        for (int i = 0; i < 8; ++i) {
            worldCorners[i] = R * localCorners[i] + center;
        }
        return worldCorners;
    }

    void DrawBevCuboid(cv::Mat &canvas,
                       const std::array<Eigen::Vector3f, 8> &corners,
                       const Eigen::Vector3f &center,
                       const cv::Scalar &color,
                       float fbl,
                       int thickness,
                       const std::string &label)
    {
        if (canvas.empty()) {
            return;
        }

        auto toCanvasPoint = [&](const Eigen::Vector3f &p) -> cv::Point {
            return cv::Point(int(p.x() / fbl + canvas.cols * 0.5f),
                             int(-p.z() / fbl + canvas.rows * 0.5f));
        };

        const int baseIndices[4] = {0, 1, 2, 3};
        std::vector<cv::Point> polygon;
        polygon.reserve(4);
        for (int idx : baseIndices) {
            polygon.push_back(toCanvasPoint(corners[idx]));
        }
        cv::polylines(canvas, polygon, true, color, thickness, cv::LINE_AA);

        const cv::Point centerPt = toCanvasPoint(center);
        const Eigen::Vector3f forward = 0.25f * (corners[1] + corners[2] + corners[5] + corners[6]) - center;
        const cv::Point headingPt = toCanvasPoint(center + forward);
        cv::arrowedLine(canvas, centerPt, headingPt, color, thickness, cv::LINE_AA, 0, 0.25);
        cv::circle(canvas, centerPt, 3, color, -1, cv::LINE_AA);

        if (!label.empty()) {
            cv::putText(canvas, label, centerPt + cv::Point(6, -6),
                        cv::FONT_HERSHEY_PLAIN, 1.0, color, 1, cv::LINE_AA);
        }
    }

    void DrawObjectObservationsOnMap(cv::Mat &canvas,
                                     const LL_SLAM::Frame *pFrame,
                                     const Eigen::Matrix4f &Twb,
                                     const Eigen::Vector3f &viewerPos,
                                     float fbl)
    {
        if (pFrame == nullptr) {
            return;
        }
        const Eigen::Matrix3f Rwb = LL_SLAM::CommonTools::T2R(Twb);
        const Eigen::Vector3f twb = LL_SLAM::CommonTools::T2t(Twb);
        Eigen::Quaternionf qwb(Rwb);
        qwb.normalize();

        for (int objectIdx = 0; objectIdx < int(pFrame->mvObjectObservations.size()); ++objectIdx) {
            const auto &obj = pFrame->mvObjectObservations[objectIdx];
            LL_SLAM::MapObject *pMapObject = ResolveTrackedMapObject(pFrame, objectIdx);
            const LL_SLAM::MapObject::ObjectState state = ResolveVisualizationState(obj, pMapObject);
            if (state == LL_SLAM::MapObject::OBJECT_STATE_STATIC) {
                continue;
            }
            const Eigen::Vector3f worldCenter = Rwb * obj.t_ref + twb;
            Eigen::Quaternionf worldRotation = qwb * obj.q_ref;
            worldRotation.normalize();
            const Eigen::Vector3f centerInViewer = worldCenter - viewerPos;
            const auto corners = BuildCuboidCorners(centerInViewer, worldRotation, obj.size);
            DrawBevCuboid(canvas,
                          corners,
                          centerInViewer,
                          GetObjectColor(state, true),
                          fbl,
                          2,
                          std::to_string(obj.track_id));
        }
    }

    void DrawMapObjectsOnMap(cv::Mat &canvas,
                             const std::vector<LL_SLAM::MapObject*> &vMapObjects,
                             const Eigen::Vector3f &viewerPos,
                             float fbl)
    {
        for (LL_SLAM::MapObject *pObj : vMapObjects) {
            if (pObj == nullptr || pObj->isBad() || !pObj->IsStatic()) {
                continue;
            }
            const Eigen::Vector3f center = pObj->GetWorldPos() - viewerPos;
            const auto corners = BuildCuboidCorners(center, pObj->GetWorldRotation(), pObj->GetSize());
            DrawBevCuboid(canvas,
                          corners,
                          center,
                          GetObjectColor(pObj->GetState(), false),
                          fbl,
                          2,
                          std::to_string(pObj->GetTrackId()));
        }
    }

    void DrawProjectedCuboidOnPreview(cv::Mat &preview,
                                      const LL_SLAM::Frame *pFrame,
                                      int camIndex,
                                      const cv::Size &sourceSize,
                                      const LL_SLAM::ObjectObservation &obj,
                                      const LL_SLAM::MapObject *pMapObject)
    {
        if (preview.empty() || sourceSize.width <= 0 || sourceSize.height <= 0) {
            return;
        }

        static const int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        const Eigen::Matrix4f Tcb = pFrame->mvTbc_cams[camIndex].inverse();
        const Eigen::Matrix3f Rcb = LL_SLAM::CommonTools::T2R(Tcb);
        const Eigen::Vector3f tcb = LL_SLAM::CommonTools::T2t(Tcb);
        const Eigen::Matrix3f &K = pFrame->mvK_cams[camIndex];
        const float nearPlane = 0.2f;

        const auto cornersBody = BuildCuboidCorners(obj.t_ref, obj.q_ref, obj.size);
        std::array<Eigen::Vector3f, 8> cornersCam;
        for (int i = 0; i < 8; ++i) {
            cornersCam[i] = Rcb * cornersBody[i] + tcb;
        }

        auto projectCameraPoint = [&](const Eigen::Vector3f &pointCam) -> cv::Point {
            const float u = K(0, 0) * pointCam.x() / pointCam.z() + K(0, 2);
            const float v = K(1, 1) * pointCam.y() / pointCam.z() + K(1, 2);
            const cv::Point2f previewPoint = ScalePointToPreview(cv::Point2f(u, v), sourceSize, preview.size());
            return cv::Point(cvRound(previewPoint.x), cvRound(previewPoint.y));
        };

        const LL_SLAM::MapObject::ObjectState state = ResolveVisualizationState(obj, pMapObject);
        const cv::Scalar color = GetObjectColor(state, true);
        int drawnEdges = 0;
        for (const auto &edge : edges) {
            Eigen::Vector3f p0 = cornersCam[edge[0]];
            Eigen::Vector3f p1 = cornersCam[edge[1]];

            if (p0.z() <= nearPlane && p1.z() <= nearPlane) {
                continue;
            }

            if (p0.z() <= nearPlane || p1.z() <= nearPlane) {
                const float t = (nearPlane - p0.z()) / (p1.z() - p0.z());
                const Eigen::Vector3f intersection = p0 + t * (p1 - p0);
                if (p0.z() <= nearPlane) {
                    p0 = intersection;
                } else {
                    p1 = intersection;
                }
            }

            cv::Point q0 = projectCameraPoint(p0);
            cv::Point q1 = projectCameraPoint(p1);
            if (cv::clipLine(cv::Rect(0, 0, preview.cols, preview.rows), q0, q1)) {
                cv::line(preview, q0, q1, color, 2, cv::LINE_AA);
                drawnEdges++;
            }
        }
        if (drawnEdges == 0) {
            return;
        }

        cv::Point labelPoint;
        bool hasLabelPoint = false;
        for (int i = 0; i < 8; ++i) {
            if (cornersCam[i].z() <= nearPlane) {
                continue;
            }
            cv::Point candidate = projectCameraPoint(cornersCam[i]);
            if (!hasLabelPoint || candidate.y < labelPoint.y) {
                labelPoint = candidate;
                hasLabelPoint = true;
            }
        }
        if (!hasLabelPoint) {
            return;
        }
        labelPoint.x = std::max(4, std::min(preview.cols - 40, labelPoint.x));
        labelPoint.y = std::max(12, std::min(preview.rows - 4, labelPoint.y));
        cv::putText(preview,
                    std::to_string(obj.track_id),
                    labelPoint + cv::Point(4, -4),
                    cv::FONT_HERSHEY_PLAIN,
                    1.0,
                    color,
                    1,
                    cv::LINE_AA);
    }

    std::vector<int> DistancetoRGB(float dis)
    {
        int h = int(6 * dis) % 360;
        int s = 100;
        int v = 100;

        if (abs(dis) < 1e-6) {
            h = 0;
            s = 0;
            v = 100;
        }

        if (h >= 360) h = 360;
        if (s >= 100) s = 100;
        if (v >= 100) v = 100;

        int i;
        int R_Color = 0;
        int B_Color = 0;
        int G_Color = 0;
        i = h / 60;
        int difs = h % 60;
        float RGB_max = v * 2.55f;
        float RGB_min = RGB_max * (100 - s) / 100.0f;
        float RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f;
        switch(i)
        {
            case 0:
                R_Color = RGB_max;
                G_Color = RGB_min + RGB_Adj;
                B_Color = RGB_min;
                break;

            case 1:
                R_Color = RGB_max - RGB_Adj;
                G_Color = RGB_max;
                B_Color = RGB_min;
                break;

            case 2:
                R_Color = RGB_min;
                G_Color = RGB_max;
                B_Color = RGB_min + RGB_Adj;
                break;

            case 3:
                R_Color = RGB_min;
                G_Color = RGB_max - RGB_Adj;
                B_Color = RGB_max;
                break;

            case 4:
                R_Color = RGB_min + RGB_Adj;
                G_Color = RGB_min;
                B_Color = RGB_max;
                break;

            default:
                R_Color = RGB_max;
                G_Color = RGB_min;
                B_Color = RGB_max - RGB_Adj;
                break;
        }

        std::vector<int> tempColor;
        tempColor.push_back(R_Color);
        tempColor.push_back(G_Color);
        tempColor.push_back(B_Color);
        return tempColor;
    }
}


namespace LL_SLAM
{
    Viewer::Viewer(System *pSystem) {
        mpSystem = pSystem;

        cv::FileNode fnSaveSnapshots = mpSystem->mSettings["Viewer.SaveKeyFrameSnapshots"];
        if (!fnSaveSnapshots.empty()) {
            mbSaveKeyFrameSnapshots = int(fnSaveSnapshots) != 0;
        }

        if (mbSaveKeyFrameSnapshots) {
            std::string sequencePath = mpSystem->mSettings["SequencePath"];
            if (!sequencePath.empty()) {
                mKeyFrameSnapshotDir = sequencePath + "/KeyFrameVisualization";
                if (!EnsureDirectory(mKeyFrameSnapshotDir)) {
                    cout << "[Viewer] Failed to create keyframe snapshot directory: " << mKeyFrameSnapshotDir << endl;
                    mbSaveKeyFrameSnapshots = false;
                }
            } else {
                mbSaveKeyFrameSnapshots = false;
            }
        }

        if (mMp4LayoutMode == MP4_LAYOUT_SURROUND_8) {
            mWidth = mSurroundWidth;
            mHeight = mSurroundHeight;
        } else {
            mWidth = mSingleWidth;
            mHeight = mSingleHeight;
        }

        cv::Size frameSize(mWidth, mHeight);
        mVideoWriter = cv::VideoWriter("LL_SLAM_MultiCamera.mp4",
                                       cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                       20,
                                       frameSize);
        if (!mVideoWriter.isOpened()) {
            cout << "[Viewer] Failed to open LL_SLAM_MultiCamera.mp4 for writing." << endl;
        }
    }

    Viewer::~Viewer() {
        if (mVideoWriter.isOpened()) {
            mVideoWriter.release();
        }
    }

    void Viewer::RequestFinish() {
        unique_lock<mutex> lock(mMutexMsg);
        mbFinishRequested = true;
    }

    void Viewer::Run() {
        while (1) {

            if (mvvImCams.empty()) {
                if (mbFinishRequested) {
                    break;
                }
                usleep(2000);
            } else {
                vector<cv::Mat> vImCams;
                Frame *pCurrentFrame;
                bool bSaveSnapshot = false;
                {
                    unique_lock<mutex> lock(mMutexMsg);
                    size_t selectedIdx = mvvImCams.size() - 1;
                    for (size_t i = mvSaveSnapshotFlags.size(); i-- > 0;) {
                        if (mvSaveSnapshotFlags[i]) {
                            selectedIdx = i;
                            break;
                        }
                    }
                    vImCams = mvvImCams[selectedIdx];
                    pCurrentFrame = mvpFrame[selectedIdx];
                    bSaveSnapshot = mvSaveSnapshotFlags[selectedIdx];
                    mvvImCams.clear();
                    mvpFrame.clear();
                    mvSaveSnapshotFlags.clear();
                }

                Visualization(vImCams, pCurrentFrame, bSaveSnapshot);

//                VisualizationYZ(vImCams, pCurrentFrame);
//
//                VisualizationXY(vImCams, pCurrentFrame);
            }
        }
    }


    void Viewer::InsertFrame(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame, bool bSaveSnapshot)
    {
        // return;
//        cout << "InsertFrame " << endl;
        unique_lock<mutex> lock(mMutexMsg);
        mvvImCams.push_back(vImCams);
        mvpFrame.push_back(pCurrentFrame);
        mvSaveSnapshotFlags.push_back(bSaveSnapshot);
//
//        vector<vector<cv::Mat>> mvvImCams;
//        vector<Frame *> mvpFrame;
    }





    void Viewer::Visualization(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame, bool bSaveSnapshot) {

        cv::Size previewSize(480, 270);
        if (mMp4LayoutMode == MP4_LAYOUT_SURROUND_8) {
            previewSize = GetSurroundPreviewSize(vImCams[0], mWidth, mHeight);
        }

        cv::Mat imShow = MakePreviewImage(vImCams[0], previewSize);
        cv::Size baseSourceSize = vImCams[0].size();

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());
        mvTopologyStatusHistory.push_back(pCurrentFrame->mCarlaTopologyStatus);


        ///////visual
        int w = mWidth;
        int h = mHeight;
        float fbl = 0.08;
//        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        DrawTopologySemanticBackground(img_map, Twbs, mvTopologyStatusHistory, t_w_viewer, fbl);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw.z() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int( Pw.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw.z() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int(-twb_i.z() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int(-twb_i.z() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        DrawMapObjectsOnMap(img_map, mpSystem->mpMap->mvpObjectObservations, t_w_viewer, fbl);
        DrawObjectObservationsOnMap(img_map, pCurrentFrame, pCurrentFrame->GetTwb(), t_w_viewer, fbl);

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int(Pw_cameravertex_i.x() / fbl + w / 2.0);
                        int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                        int Pj_u = int(Pw_cameravertex_j.x() / fbl + w / 2.0);
                        int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }
        {
            int nStaticObjects = 0;
            int nDynamicObjects = 0;
            int nUnknownObjects = 0;
            std::vector<LL_SLAM::MapObject*> vMapObjects;
            {
                unique_lock<mutex> lock(mpSystem->mpMap->mMutexUpdate);
                vMapObjects = mpSystem->mpMap->mvpObjectObservations;
            }
            for (LL_SLAM::MapObject *pObj : vMapObjects) {
                if (pObj == nullptr || pObj->isBad()) {
                    continue;
                }
                if (pObj->IsStatic()) {
                    nStaticObjects++;
                } else if (pObj->IsDynamic()) {
                    nDynamicObjects++;
                } else {
                    nUnknownObjects++;
                }
            }
            std::stringstream s;
            s << "obj obs : " << pCurrentFrame->mvObjectObservations.size()
              << " static : " << nStaticObjects
              << " dynamic : " << nDynamicObjects
              << " unknown : " << nUnknownObjects
              << " map obj : " << vMapObjects.size();
            cv::putText(img_map, s.str(), cv::Point(12, 24), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
        }
        DrawTopologySemanticLegend(img_map, pCurrentFrame);

        //////////////////////////////////////////////////////////////////////////
//        //debug
//
//
//        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
//            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;
//
//            int Pi_u = int( Pw.x() / fbl + w / 2.0);
//            int Pi_v = int(-Pw.z() / fbl + h / 2.0);
//            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);
//
//        }
//
//
//



        //////////////////////////////////////////////////////////////////////////
        if (mMp4LayoutMode == MP4_LAYOUT_SURROUND_8) {
            vector<int> vImShowIndex = SelectSurroundPreviewCameras(pCurrentFrame);
            if (vImShowIndex.back() >= pCurrentFrame->mNumCam) {
                vImShowIndex = {0};
            } else if (vImCams.size() > 2 && vImShowIndex.size() >= 3 && vImCams[1].datastart == vImCams[2].datastart) {
                vImShowIndex = {0};
            }

            vector<pair<int, int>> vImShowPos = {{0, 0},
                                                 {int(img_map.cols * 0.5 - imShow.cols * 0.5), 0},
                                                 {int(img_map.cols - imShow.cols), 0},
                                                 {int(img_map.cols - imShow.cols), int(img_map.rows * 0.5 - imShow.rows * 0.5)},
                                                 {int(img_map.cols - imShow.cols), int(img_map.rows - imShow.rows)},
                                                 {int(img_map.cols * 0.5 - imShow.cols * 0.5), int(img_map.rows - imShow.rows)},
                                                 {0, int(img_map.rows - imShow.rows)},
                                                 {0, int(img_map.rows * 0.5 - imShow.rows * 0.5)}};

            for (int cami = 0; cami < vImShowIndex.size() && cami < vImShowPos.size(); cami++)
            {
                int CamIndex = vImShowIndex[cami];
                if (CamIndex >= vImCams.size() || CamIndex >= pCurrentFrame->mvCamKeysUn.size() || CamIndex >= pCurrentFrame->mvMapPoints.size()) {
                    continue;
                }

                cv::Mat imShowCami = MakePreviewImage(vImCams[CamIndex], imShow.size());
                vector<cv::KeyPoint> vCamKeys = pCurrentFrame->mvCamKeysUn[CamIndex];
                cv::Size cameraSourceSize = vImCams[CamIndex].size();
                int keyCount = vCamKeys.size();
                if (keyCount > pCurrentFrame->mvMapPoints[CamIndex].size()) {
                    keyCount = pCurrentFrame->mvMapPoints[CamIndex].size();
                }

                for (int i = 0; i < keyCount; i++) {
                    cv::Point2f previewPoint = ScalePointToPreview(vCamKeys[i].pt, cameraSourceSize, imShowCami.size());

                    if (pCurrentFrame->mvMapPoints[CamIndex][i] != NULL) {
                        float maxd = 60;
                        float centerX = imShowCami.cols * 0.5;
                        float centerY = imShowCami.rows * 0.5;
                        cv::Point2f point = previewPoint;
                        float d = sqrt((point.x - centerX) * (point.x - centerX) + (point.y - centerY) * (point.y - centerY)) * maxd / (max(centerX, centerY));
                        std::vector<int> color = DistancetoRGB(d);
                        int ringWidth = 2;
                        int ringMin = 4;
                        int ringMax = ringWidth + ringMin;
                        cv::circle(imShowCami, previewPoint, ringMax, cv::Scalar(255, 255, 255), -1);
                        cv::circle(imShowCami, previewPoint, ringMin, cv::Scalar(color[2], color[1], color[0]), -1);
                    }
                }

                for (int objectIdx = 0; objectIdx < int(pCurrentFrame->mvObjectObservations.size()); ++objectIdx) {
                    const auto &obj = pCurrentFrame->mvObjectObservations[objectIdx];
                    LL_SLAM::MapObject *pMapObject = ResolveTrackedMapObject(pCurrentFrame, objectIdx);
                    DrawProjectedCuboidOnPreview(imShowCami, pCurrentFrame, CamIndex, cameraSourceSize, obj, pMapObject);
                }

                CopyPreviewToCanvas(imShowCami, img_map, vImShowPos[cami].first, vImShowPos[cami].second);
            }
        } else {
            for (int objectIdx = 0; objectIdx < int(pCurrentFrame->mvObjectObservations.size()); ++objectIdx) {
                const auto &obj = pCurrentFrame->mvObjectObservations[objectIdx];
                LL_SLAM::MapObject *pMapObject = ResolveTrackedMapObject(pCurrentFrame, objectIdx);
                DrawProjectedCuboidOnPreview(imShow, pCurrentFrame, 0, baseSourceSize, obj, pMapObject);
            }
            CopyPreviewToCanvas(imShow, img_map, 0, 0);
        }

        if (bSaveSnapshot) {
            SaveKeyFrameVisualization(img_map, pCurrentFrame);
        }

        if (mVideoWriter.isOpened()) {
            mVideoWriter.write(img_map);
        }

        // 将所有像素设置为白色（最高亮度值）
        // cv::imshow("img_map", img_map);
        // cv::waitKey(2);
    }


    void Viewer::SaveKeyFrameVisualization(const cv::Mat &imComposite, Frame *pCurrentFrame)
    {
        if (!mbSaveKeyFrameSnapshots || pCurrentFrame == nullptr || imComposite.empty() || mKeyFrameSnapshotDir.empty()) {
            return;
        }

        std::ostringstream oss;
        oss << mKeyFrameSnapshotDir << "/" << std::setfill('0') << std::setw(6) << pCurrentFrame->mnId << ".jpg";
        const std::string outputPath = oss.str();
        if (!cv::imwrite(outputPath, imComposite)) {
            cout << "[Viewer] Failed to save keyframe visualization: " << outputPath << endl;
        }
    }

    void Viewer::VisualizationYZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) {

        cv::Mat imShow = vImCams[0].clone();
        vector<cv::KeyPoint> vKeys = pCurrentFrame->mvCamKeysUn[0];
        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//        cv::resize(imShow, imShow, cv::Size(320, 180));
        cv::resize(imShow, imShow, cv::Size(480, 270));
//        cv::imshow("imShow", imShow);

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());
        mvTopologyStatusHistory.push_back(pCurrentFrame->mCarlaTopologyStatus);


        ///////visual
//        int w = 1000;
        int w = 2500;
        int h = 1400;
//        float fbl = 0.03;
        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        DrawTopologySemanticBackground(img_map, Twbs, mvTopologyStatusHistory, t_w_viewer, fbl);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int(-Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int(-Pw.z() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int(-Pw.z() / fbl + w / 2.0);
                int Pi_v = int( Pw.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int(-twb_i.z() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int(-twb_i.z() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                        int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                        int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
                        int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }


        //////////////////////////////////////////////////////////////////////////
        //debug


        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;

            int Pi_u = int(-Pw.z() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);

        }






        //////////////////////////////////////////////////////////////////////////


        imShow.copyTo(img_map(cv::Rect(max(int(0), 0),
                                       max(int(0), 0),
                                       (min(int(imShow.cols), img_map.cols) - max(0, 0)),
                                       (min(int(imShow.rows), img_map.rows) - max(0, 0)))));

        // 将所有像素设置为白色（最高亮度值）
        cv::imshow("img_map_YZ", img_map);
//


        cv::waitKey(2);
    }


    void Viewer::VisualizationXY(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) {

        cv::Mat imShow = vImCams[0].clone();
        vector<cv::KeyPoint> vKeys = pCurrentFrame->mvCamKeysUn[0];
        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//        cv::resize(imShow, imShow, cv::Size(320, 180));
        cv::resize(imShow, imShow, cv::Size(480, 270));
//        cv::imshow("imShow", imShow);

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());
        mvTopologyStatusHistory.push_back(pCurrentFrame->mCarlaTopologyStatus);


        ///////visual
//        int w = 1000;
        int w = 2500;
        int h = 1400;
        float fbl = 0.03;
//        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        DrawTopologySemanticBackground(img_map, Twbs, mvTopologyStatusHistory, t_w_viewer, fbl);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int( Pw.x() / fbl + w / 2.0);
                int Pi_v = int( Pw.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                        int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                        int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                        int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }


        //////////////////////////////////////////////////////////////////////////
        //debug


        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;

            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);

        }






        //////////////////////////////////////////////////////////////////////////


        imShow.copyTo(img_map(cv::Rect(max(int(0), 0),
                                       max(int(0), 0),
                                       (min(int(imShow.cols), img_map.cols) - max(0, 0)),
                                       (min(int(imShow.rows), img_map.rows) - max(0, 0)))));

        // 将所有像素设置为白色（最高亮度值）
        cv::imshow("img_map_XY", img_map);
        
        cv::waitKey(2);
    }

} //namespace ORB_SLAM
