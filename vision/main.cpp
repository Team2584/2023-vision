#include "main.h"
#include "Cameras.h"
#include "detection.h"
#include "globals.h"
#include "graphics_helpers.h"
#include "pose_estimation.h"

using namespace std;
using namespace cv;
using mjpgs = nadjieb::MJPEGStreamer;

int total_hamm_hist[HAMM_HIST_MAX];
int hamm_hist[HAMM_HIST_MAX];

int cone_min_hue;
int cone_max_hue;
int cone_min_sat;
int cone_max_sat;
int cone_min_val;
int cone_max_val;

int cube_min_hue;
int cube_max_hue;
int cube_min_sat;
int cube_max_sat;
int cube_min_val;
int cube_max_val;

bool has_depth = true;
bool has_tagCam = true;

int main()
{
    /**********************************************************************************************
     * AprilTags Setup *
     *******************/

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    tf = tag16h5_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, HAMMING_NUMBER);

    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma = QUAD_SIGMA;
    td->nthreads = NTHREADS;
    td->debug = APRIL_DEBUG;
    td->refine_edges = REFINE_EDGES;

    memset(total_hamm_hist, 0, sizeof(int) * HAMM_HIST_MAX);

    /**********************************************************************************************
     * Network Tables Setup *
     ************************/

    // Create networktables instan8ce and a table for vision
    nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();

    // Setup networktable client
    nt_inst.StartClient4("jetson client");
    nt_inst.SetServerTeam(2584);
    nt_inst.StartDSClient();
    nt_inst.SetServer("host", NT_DEFAULT_PORT4);

    // Create tables
    shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");

    // Make a sanity check topic and an entry to publish/read from it; set initial
    // value
    nt::IntegerTopic sanitycheck = visionTbl->GetIntegerTopic("sanitycheck");
    nt::IntegerEntry sanitycheckEntry = sanitycheck.GetEntry(0, {.periodic = 0.01});
    sanitycheckEntry.Set(1);

    // Connected topic
    nt::BooleanTopic connectedTopic = visionTbl->GetBooleanTopic("connected");
    nt::BooleanSubscriber connectedSub = connectedTopic.Subscribe(false);

    // Other vision topics
    nt::DoubleArrayTopic cube_tag_Topic = visionTbl->GetDoubleArrayTopic("cubeTag");
    nt::DoubleArrayEntry cube_tag_Entry = cube_tag_Topic.GetEntry({});

    nt::DoubleArrayTopic substation_tag_Topic = visionTbl->GetDoubleArrayTopic("substationTag");
    nt::DoubleArrayEntry substation_tag_Entry = substation_tag_Topic.GetEntry({});

    nt::BooleanTopic see_cones_Topic = visionTbl->GetBooleanTopic("seeCones");
    nt::BooleanSubscriber see_cones_Sub = see_cones_Topic.Subscribe({true});

    nt::BooleanTopic see_cubes_Topic = visionTbl->GetBooleanTopic("seeCubes");
    nt::BooleanSubscriber see_cubes_Sub = see_cubes_Topic.Subscribe({true});

    nt::BooleanTopic see_cube_tags_Topic = visionTbl->GetBooleanTopic("seeCubeTags");
    nt::BooleanSubscriber see_cube_tags_Sub = see_cube_tags_Topic.Subscribe({true});

    nt::BooleanTopic see_substation_tags_Topic = visionTbl->GetBooleanTopic("seeSubstationTags");
    nt::BooleanSubscriber see_substation_tags_Sub = see_substation_tags_Topic.Subscribe({true});

    nt::BooleanTopic in_match_Topic = visionTbl->GetBooleanTopic("inMatch");
    nt::BooleanSubscriber in_match_Sub = in_match_Topic.Subscribe({false});

    nt::DoubleArrayTopic cone_pos_Topic = visionTbl->GetDoubleArrayTopic("conePos");
    nt::DoubleArrayPublisher cone_pos_Pub = cone_pos_Topic.Publish();

    nt::DoubleArrayTopic cube_pos_Topic = visionTbl->GetDoubleArrayTopic("cubePos");
    nt::DoubleArrayPublisher cube_pos_Pub = cube_pos_Topic.Publish();

    nt::DoubleArrayTopic pole_pos_Topic = visionTbl->GetDoubleArrayTopic("polePos");
    nt::DoubleArrayPublisher pole_pos_Pub = pole_pos_Topic.Publish();

    nt::BooleanTopic has_cone_Topic = visionTbl->GetBooleanTopic("hasCone");
    nt::BooleanEntry has_cone_Entry = has_cone_Topic.GetEntry({true});

    /**********************************************************************************************
     * Camera & Stream Setup *
     *************************/

    // depthCamera *depth;
    // try
    //{
    depthCamera depth(DEPTH_RED, 640, 480, 60);
    // depth = &depth_obj;
    //}
    /*
    catch (const rs2::error &err)
    {
        if (!in_match_Sub.Get())
        {
            return (7);
        }
        cout << "RS2 ERROR CAUGHT: " << err.what() << endl;
        has_depth = false;
    }
    */

    if (has_depth)
    {
        depth.setPosOffset(1 / INCH, 5 / INCH, 23.5 / INCH, 0, -0.349);
        // Red camera settings
        depth.setCamParams(599, 600, 334, 236);
    }

    // depthCamera *tagCam;
    /*
    try
    {
        depthCamera tagCam(DEPTH_BLUE, 640, 480, 60);
        // tagCam = &tagCam_obj;
    }
    catch (const rs2::error &err)
    {
        if (!in_match_Sub.Get())
        {
            return (7);
        }
        cout << "RS2 ERROR CAUGHT: " << err.what() << endl;
        has_tagCam = false;
    }
    */

#ifdef hasTagCam
    {
        tagCam.setPosOffset(1 / INCH, -2 / INCH, 23.5 / INCH, 0, -0.349);
        tagCam.setCamParams(608, 608, 323, 245);
        tagCam.setDistCoeffs(0.09116903370720442, 0.2567349843314421, -0.003936586357063021,
                             0.001658039412119442, -1.633408316803933);
#endif

        vector<int> params = {IMWRITE_JPEG_QUALITY, 80};
        mjpgs streamer;
        streamer.start(5802);

        /**********************************************************************************************
         * GET FILTER PARAMETERS *
         *************************/
        vector<int> coneParams;
        vector<int> cubeParams;

        string line;
        ifstream coneFile("/home/patriotrobotics/Documents/vision/cone-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (coneFile)
                getline(coneFile, line);
            if (line != "")
                coneParams.push_back(stoi(line));
        }
        if (coneParams.size() > 0)
        {
            cone_min_hue = coneParams[0];
            cout << "cone_min_hue " << cone_min_hue << endl;
            cone_max_hue = coneParams[1];
            cout << "cone_max_hue " << cone_max_hue << endl;
            cone_min_sat = coneParams[2];
            cout << "cone_min_sat " << cone_min_sat << endl;
            cone_max_sat = coneParams[3];
            cout << "cone_max_sat " << cone_max_sat << endl;
            cone_min_val = coneParams[4];
            cout << "cone_min_val " << cone_min_val << endl;
            cone_max_val = coneParams[5];
            cout << "cone_max_val " << cone_max_val << endl << endl;
        }

        ifstream cubeFile("/home/patriotrobotics/Documents/vision/cube-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (coneFile)
                getline(cubeFile, line);
            if (line != "")
                cubeParams.push_back(stoi(line));
        }
        if (cubeParams.size() > 0)
        {
            cube_min_hue = cubeParams[0];
            cout << "cube_min_hue " << cube_min_hue << endl;
            cube_max_hue = cubeParams[1];
            cout << "cube_max_hue " << cube_max_hue << endl;
            cube_min_sat = cubeParams[2];
            cout << "cube_min_sat " << cube_min_sat << endl;
            cube_max_sat = cubeParams[3];
            cout << "cube_max_sat " << cube_max_sat << endl;
            cube_min_val = cubeParams[4];
            cout << "cube_min_val " << cube_min_val << endl;
            cube_max_val = cubeParams[5];
            cout << "cube_max_val " << cube_max_val << endl << endl << endl;
        }

        /**********************************************************************************************
         * THE LOOP *
         ************/
        int counter = 2;
        double poseNum = 0;
        double coneNum = 0;

        while (true)
        {
            chrono::time_point loop_start = chrono::steady_clock::now();

            if (!connectedSub.Get())
            {
                printf("Reconnecting...\n");
                nt_inst.StartClient4("jetson client");
                nt_inst.SetServerTeam(2584);
                nt_inst.StartDSClient();
                nt_inst.SetServer("host", NT_DEFAULT_PORT4);
            }

            errno = 0;
            memset(hamm_hist, 0, sizeof(hamm_hist));

            // Make sure networktables is working
            sanitycheckEntry.Set(counter);
            counter++;

            if (has_depth)
                depth.getFrame();
            chrono::time_point frameTime = chrono::steady_clock::now();

#ifdef hasTagCam
            tagCam.getFrame();
#endif

            if (errno == EAGAIN)
            {
                printf("Unable to create the %d threads requested.\n", td->nthreads);
                continue;
            }

            if (has_depth)
            {
                if (see_cube_tags_Sub.Get())
                {
                    std::vector<robot_position> poses =
                        getPoses(depth.grayFrame, depth.colorFrame, &(depth.info), td);

                    for (unsigned int i = 0; i < poses.size(); i++)
                    {
                        robot_position pos = poses[i];
                        cout << "X: " << pos.x * INCH << endl;
                        cout << "Y: " << pos.y * INCH << endl;
                        cout << "Z: " << pos.z * INCH << endl << endl;
                        cout << "Theta: " << pos.theta << endl;

                        double micros = time_since(frameTime);
                        vector<double> poseVector = {pos.x,     pos.y,  pos.z,
                                                     pos.theta, micros, poseNum};
                        cube_tag_Entry.Set(poseVector);
                        nt_inst.Flush();
                        poseNum++;
                    }
                }
            }

#ifdef hasTagCam
            {
                if (see_substation_tags_Sub.Get())
                {
                    std::vector<robot_position> poses =
                        getPoses(tagCam.grayFrame, tagCam.colorFrame, &tagCam.info, td);

                    for (unsigned int i = 0; i < poses.size(); i++)
                    {
                        robot_position pos = poses[i];
                        cout << "X: " << pos.x * INCH << endl;
                        cout << "Y: " << pos.y * INCH << endl;
                        cout << "Z: " << pos.z * INCH << endl << endl;
                        cout << "Theta: " << pos.theta << endl;

                        double micros = time_since(frameTime);
                        vector<double> poseVector = {pos.x,     pos.y,  pos.z,
                                                     pos.theta, micros, poseNum};
                        substation_tag_Entry.Set(poseVector);
                        nt_inst.Flush();
                        poseNum++;
                    }
                }
#endif

            if (has_depth)
            {
                if (see_cones_Sub.Get())
                {
                    // Print & send cone info
                    pair<double, double> conePos = depth.findCones();
                    cout << "Cone X: " << conePos.first << endl;
                    cout << "Cone Y: " << conePos.second << endl;
                    double micros = time_since(frameTime);
                    vector<double> coneVector = {conePos.first, conePos.second, micros, coneNum};
                    cone_pos_Pub.Set(coneVector);
                }

                if (see_cubes_Sub.Get())
                {
                    // Print & send cube info
                    pair<double, double> cubePos = depth.findCubes();
                    cout << "Cube X: " << cubePos.first << endl;
                    cout << "Cube Y: " << cubePos.second << endl << endl;
                    double micros = time_since(frameTime);
                    vector<double> cubeVector = {cubePos.first, cubePos.second, micros, coneNum};
                    cube_pos_Pub.Set(cubeVector);
                }
            }

            // Print & send pole info
            /*
            pair<double, double> polePos = depth.findPoles();
            cout << "Pole X: " << polePos.first << endl;
            cout << "Pole Y: " << polePos.second << endl;
            ms = time_since(frameTime);
            vector<double> poleVector = {polePos.first, polePos.second, ms, id}
            pole_pos_Pub.Set(poleVector);
            */

            nt_inst.Flush();

            // Graphics stuff
            // drawMargins(depth.colorFrame);

            // streamer
            if (has_depth)
            {
                vector<uchar> buf_bgr;
                imencode(".jpg", depth.colorFrame, buf_bgr, params);
                streamer.publish("/colorFrame", string(buf_bgr.begin(), buf_bgr.end()));
            }

#ifdef hasTagCam
            {
                vector<uchar> buf_tags;
                imencode(".jpg", tagCam.colorFrame, buf_tags, params);
                streamer.publish("/tagCam", string(buf_tags.begin(), buf_tags.end()));
#endif

                // Make sure each loop takes at least 10 ms (for streamer library)
                auto loop_time = chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - loop_start);
                if (loop_time.count() < 10)
                    this_thread::sleep_for(chrono::milliseconds(10 - loop_time.count()));
            }

            apriltag_detector_destroy(td);
            tag16h5_destroy(tf);
            streamer.stop();

            return 0;
        }

        // Return time elapsed since passed time_point in microseconds
        double time_since(std::chrono::time_point<std::chrono::steady_clock> start)
        {
            chrono::time_point end = chrono::steady_clock::now();
            auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
            return (double)duration.count();
        }
