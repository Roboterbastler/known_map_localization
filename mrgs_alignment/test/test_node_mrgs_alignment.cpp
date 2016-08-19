#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mrgs_alignment/align.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_server/image_loader.h>

class MrgsAlignmentFixture: public ::testing::Test {
protected:
	static void SetUpTestCase() {
		ros::NodeHandle nodeHandle;

		nav_msgs::GetMapResponse map1_resp;
		nav_msgs::GetMapResponse map1_90cw_resp;
		nav_msgs::GetMapResponse map1_180_resp;

		std::string package_path = ros::package::getPath("mrgs_alignment");

		try {
			// default threshold values are taken from the map_server package
			map_server::loadMapFromFile(&map1_resp, (package_path + "/test/data/map1.png").c_str(), 1.0, false, 0.65,
								0.196, origin, TRINARY);
			map_server::loadMapFromFile(&map1_90cw_resp, (package_path + "/test/data/map1_90cw.png").c_str(), 1.0, false, 0.65,
								0.196, origin, TRINARY);
			map_server::loadMapFromFile(&map1_180_resp, (package_path + "/test/data/map1_180.png").c_str(), 1.0, false, 0.65,
								0.196, origin, TRINARY);
		} catch (std::runtime_error &err) {
			std::cerr << "Loading test maps failed: " << err.what() << std::endl;
			FAIL();
		}

		map1 = map1_resp.map;
		map1 = map1_90cw_resp.map;
		map1 = map1_180_resp.map;

		client = nodeHandle.serviceClient<mrgs_alignment::align>("/align");

		if(!client.exists()) {
			std::cerr << "Service is not available" << std::endl;
			FAIL();
		}

		// disable logging during testing
		//ros::console::shutdown();
	}

	// access grid's values
	static int8_t mapAt(const nav_msgs::OccupancyGrid &map, unsigned int row, unsigned int column) {
		if(row < map.info.height && column < map.info.width) {
			assert(row * column < map.data.size());

			return map.data.at(row * map.info.width + column);
		} else {
			throw std::runtime_error("Accessing a cell outside of the grid.");
		}
	}

	static ros::ServiceClient client;
	static mrgs_alignment::align srv;
	static double origin[3];
	static nav_msgs::OccupancyGrid map1;
	static nav_msgs::OccupancyGrid map1_90cw;
	static nav_msgs::OccupancyGrid map1_180;
};

// define static members
ros::ServiceClient MrgsAlignmentFixture::client;
mrgs_alignment::align MrgsAlignmentFixture::srv;
double MrgsAlignmentFixture::origin[3] = { 0.0, 0.0, 0.0 };
nav_msgs::OccupancyGrid MrgsAlignmentFixture::map1;
nav_msgs::OccupancyGrid MrgsAlignmentFixture::map1_90cw;
nav_msgs::OccupancyGrid MrgsAlignmentFixture::map1_180;

TEST_F(MrgsAlignmentFixture, identicalMaps) {
	srv.request.number_of_hypotheses = 1;
	srv.request.map1 = map1;
	srv.request.map2 = map1;

	// call align service
	ASSERT_TRUE(client.call(srv));
	ASSERT_GE(srv.response.success_coefficient, 0.0);

	std::vector<geometry_msgs::TransformStamped> transforms = srv.response.transforms;
	ASSERT_EQ(1, transforms.size());
	geometry_msgs::Transform &transform = transforms.at(0).transform;

	EXPECT_FLOAT_EQ(0.0, transform.translation.x);
	EXPECT_FLOAT_EQ(0.0, transform.translation.y);
	EXPECT_FLOAT_EQ(0.0, transform.translation.z);
	EXPECT_FLOAT_EQ(0.0, transform.rotation.x);
	EXPECT_FLOAT_EQ(0.0, transform.rotation.y);
	EXPECT_FLOAT_EQ(0.0, transform.rotation.z);
	EXPECT_FLOAT_EQ(1.0, transform.rotation.w);
}

// Run all the tests
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "known_map_localization");

	return RUN_ALL_TESTS();
}
