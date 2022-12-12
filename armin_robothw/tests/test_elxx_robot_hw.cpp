#include <gtest/gtest.h>
#include "armin_robothw/elxx.h"
#include "armin_robothw/ecn.h"

class ELXXTestInternals : public ::testing::Test
{
protected:
	void SetUp()
	{
		Elxx_Region_m2s a;
		Elxx_Region_s2m b;
		EtherCATNetwork enc;
		elxx = new ELXX(a, b, enc, 0);
		elxx->outCand[0]= 0xF0;
		elxx->outCand[1]= 0xAA;
	}
	void TearDown()
	{
		delete elxx;
	}
	ELXX *elxx;
};

TEST_F(ELXXTestInternals, powerOnOutOfBounds)
{
	bool rv;
	rv = elxx->powerOnChannel(-1);
	EXPECT_EQ(rv, false);
	EXPECT_EQ(elxx->outCand[0], 0xF0);
	EXPECT_EQ(elxx->outCand[1], 0xAA);

	rv = elxx->powerOnChannel(20);
	EXPECT_EQ(rv, false);
	EXPECT_EQ(elxx->outCand[0], 0xF0);
	EXPECT_EQ(elxx->outCand[1], 0xAA);

	rv = elxx->powerOnChannel(16);
	EXPECT_EQ(rv, false);
	EXPECT_EQ(elxx->outCand[0], 0xF0);
	EXPECT_EQ(elxx->outCand[1], 0xAA);
}

TEST_F(ELXXTestInternals, powerOnCandChanges)
{
	bool rv;
	rv = elxx->powerOnChannel(0);
	EXPECT_EQ(rv, true);
	EXPECT_EQ(elxx->outCand[0], 0xF1);
	EXPECT_EQ(elxx->outCand[1], 0xAA);

	rv = elxx->powerOnChannel(7);
	EXPECT_EQ(rv, true);
	EXPECT_EQ(elxx->outCand[0], 0xF1);
	EXPECT_EQ(elxx->outCand[1], 0xAA);

	rv = elxx->powerOnChannel(8);
	EXPECT_EQ(rv, true);
	EXPECT_EQ(elxx->outCand[0], 0xF1);
	EXPECT_EQ(elxx->outCand[1], 0xAB);

	rv = elxx->powerOnChannel(14);
	EXPECT_EQ(rv, true);
	EXPECT_EQ(elxx->outCand[0], 0xF1);
	EXPECT_EQ(elxx->outCand[1], 0xEB);
}

int main(int argc, char *argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

