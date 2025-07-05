#include <SkipList.hpp>
#include <catch2/catch_amalgamated.hpp>
#include <vector>

namespace {
namespace proj2 = shindler::ics46::project2;

/*
NOTE:  these are not intended as exhaustive tests.
     You should make your own additional tests
     VERY IMPORTANT:  if your SkipList is not templated, or if
            it is not built as a linked structure, your score for this project
            will be zero.  Be sure your SkipList compiles and runs
            with non-numeric data types.


NOTE ALSO:  "Not Required" does not mean "optional" or "extra credit."
"Not required" only means that we will still grade your project, even
if these exact tests do not pass.
*/

TEST_CASE("SkipList:SimpleHeightsTest:ExpectEqualsPrecalulatedHeights",
          "[Sample][SkipList][Height]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const std::vector<unsigned> EXPECTED_HEIGHTS = {1, 2, 1, 3, 1,
                                                    2, 1, 4, 1, 2};

    proj2::SkipList<unsigned, unsigned> skipList;
    std::vector<unsigned> heights;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        std::optional<size_t> elementsAccessed = skipList.insert(i, i);
        CHECK(elementsAccessed.has_value());
        heights.push_back(skipList.height(i));
    }

    REQUIRE(heights == EXPECTED_HEIGHTS);
}

TEST_CASE("SkipList:SimpleFindTest:ExpectValueInsertedEqualsValueFound",
          "[Sample][SkipList][InsertFind]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const unsigned int VALUE_OFFSET = 100;

    proj2::SkipList<unsigned, unsigned> skipList;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, (VALUE_OFFSET + i));
    }

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        auto [value, elementsAccessed] = skipList.find(i);
        REQUIRE((i + VALUE_OFFSET) == value);
    }
}

TEST_CASE("SkipList:SimpleLargestAndSmallest:ExpectIsSmallestAndIsLargestTrue",
          "[Sample][SkipList][LargestSmallest]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const unsigned int VALUE_OFFSET = 100;

    proj2::SkipList<unsigned, unsigned> skipList;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        std::optional<size_t> elementsAccessed =
            skipList.insert(i, (VALUE_OFFSET + i));
        CHECK(elementsAccessed.has_value());
    }

    REQUIRE(skipList.isSmallestKey(0));
    REQUIRE(skipList.isLargestKey(9));
}

TEST_CASE("SkipList:InvolvedHeightsTest:ExpectEqualsPecalculatedHeights",
          "[Sample][SkipList][Height]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const unsigned MAGIC_VAL = 255;
    const std::vector<unsigned> EXPECTED_HEIGHTS = {1, 2, 1, 3, 1, 2,
                                                    1, 4, 1, 2, 12};

    proj2::SkipList<unsigned, unsigned> skipList;
    std::vector<unsigned> heights;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, i);
        heights.push_back(skipList.height(i));
    }

    // The coinFlip function will always return heads
    // for 255 regardless of the current layer.
    // You can use this value to test your threshold for halting
    // the insertion procedure. If this results in an infinite loop inside
    // of your insert function you have not implemented a cutoff threshold.
    skipList.insert(MAGIC_VAL, MAGIC_VAL);

    heights.push_back(skipList.height(MAGIC_VAL));

    // The expected height for 255 is 12 because there are fewer than 16 nodes
    // in the skip list when 12 is added.
    REQUIRE(heights == EXPECTED_HEIGHTS);

    // At this point there should be 13 layers
    // (because the fast lane is not included in the height calculation).
    REQUIRE(skipList.layers() == 13);
}

TEST_CASE("SkipList:LayersTest:Expect16", "[Sample][SkipList][Layers]") {
    const unsigned int NUMBER_OF_ELEMENTS = 16;
    const unsigned MAGIC_VAL = 255;
    const std::vector<unsigned> EXPECTED_HEIGHTS = {1, 2, 1, 3, 1, 2, 1, 4, 1,
                                                    2, 1, 3, 1, 2, 1, 5, 15};

    proj2::SkipList<unsigned, unsigned> skipList;
    std::vector<unsigned> heights;

    // First insert 16 values into the skip list [0, 15].
    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, i);
        heights.push_back(skipList.height(i));
    }

    // Same value used in SimpleHeightsTest for testing the threshold.
    skipList.insert(MAGIC_VAL, MAGIC_VAL);

    heights.push_back(skipList.height(MAGIC_VAL));

    // The expected height for 255 is 15 because 3 * ceil(log_2(17)) = 15
    // meaning the maximum height of any node should be 15 for a skip list with
    // 17 nodes.
    REQUIRE(heights == EXPECTED_HEIGHTS);

    // At this point there should be 16 layers
    // (because the fast lane is not included in the height calculation).
    REQUIRE(skipList.layers() == 16);
}

// These are stubs, suggested tests for you to implement yourself
// to ensure your solution works correctly

TEST_CASE("SkipList:TestStubs:ComplexEraseTest", "[SkipList][Stub]") {
    REQUIRE(false);
}

TEST_CASE("SkipList:TestStubs:Test10000InsertsAndFinds", "[SkipList][Stub]") {
    REQUIRE(false);
}

TEST_CASE("SkipList:TestStubs:ValidateNumberElementsAccessedFind",
          "[SkipList][Stub]") {
    REQUIRE(false);
}

TEST_CASE("SkipList:TestStubs:ValidateNumberElementsAccessedInsert",
          "[SkipList][Stub]") {
    REQUIRE(false);
}

TEST_CASE("SkipList:TestStubs:TestFailedInsertCase", "[SkipList][Stub]") {
    // Hint: std::nullopt
    REQUIRE(false);
}

TEST_CASE("SkipList:TestStubs:Test100000InsertsAndFinds",
          "[SkipList][Stub][Slow]") {
    REQUIRE(false);
}

// There are more things that could be tested for this project. Hopefully the
// above helps to give you an idea of what can be made a test.

}  // namespace
