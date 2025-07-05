#include <SkipList.hpp>
#include <catch2/catch_amalgamated.hpp>
#include <vector>
#include <optional>
#include <cstdint>
#include <cmath>
#include <cstddef>

namespace {
namespace proj2 = shindler::ics46::project2;

/*
NOTE:  these are not intended as exhaustive tests.
     You should make your own additional tests
     VERY IMPORTANT:  if your SkipList is not templated, or if
            it is not built as a linked structure, your score for this project
            will be zero.  Be sure your SkipList compiles and runs
            with non-numeric data types.


IF YOUR CODE DOES NOT PASS EVERY TEST IN THIS FILE, YOUR SCORE WILL BE A ZERO.


*/

TEST_CASE("SkipList:CreatedBasics:ExpectDefaults", "[Required][SkipList]") {
    proj2::SkipList<unsigned, unsigned> skipList;

    REQUIRE(2 == skipList.layers());
    REQUIRE(skipList.empty());
    REQUIRE(0 ==
            skipList.size());  // NOLINT(readability-container-size-empty) we
                               // use this because we also want to ensure that
                               // the size is 0 along with empty being zero
    REQUIRE(skipList.empty());
}

TEST_CASE("SkipList:InsertTestUnsignedInts:ExpectFoundValueEqualsInsertedValue",
          "[Required][SkipList]") {
    const unsigned int INSERT_KEY = 3;
    const unsigned int INSERT_VALUE = 5;

    proj2::SkipList<unsigned, unsigned> skipList;

    skipList.insert(INSERT_KEY, INSERT_VALUE);

    REQUIRE(skipList.find(INSERT_KEY).first == INSERT_VALUE);
}

TEST_CASE("SkipList:InsertTestStrings:ExpectFoundValueEqualsInsertedValue",
          "[Required][SkipList]") {
    proj2::SkipList<std::string, std::string> skipList;

    skipList.insert("Shindler", "ICS 46");

    REQUIRE(skipList.find("Shindler").first == "ICS 46");
}

TEST_CASE("SkipList:SimpleNextAndPrev:ExpectCorrectNextAndPrev",
          "[Required][SkipList]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;

    proj2::SkipList<unsigned, unsigned> skipList;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, i);
    }

    for (unsigned i = 1; i < NUMBER_OF_ELEMENTS - 1; i++) {
        REQUIRE(skipList.previousKey(i) == (i - 1));
        REQUIRE(skipList.nextKey(i) == (i + 1));
    }
}

TEST_CASE("SkipList:FindNodeTraversalLength", "[Required][SkipList]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;

    proj2::SkipList<unsigned, unsigned> skipList;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, i);
    }

    auto [value, elementsAccessed]  = skipList.find(6);

    REQUIRE(elementsAccessed == 8);
}

TEST_CASE("SkipList:InsertNodeTraversalLength", "[Required][SkipList]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const unsigned int TEST_NUM = 91;

    proj2::SkipList<unsigned, unsigned> skipList;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, i);
    }

    auto result = skipList.insert(TEST_NUM, TEST_NUM);

    REQUIRE(*result == 7);

    auto nullresult = skipList.insert(TEST_NUM, TEST_NUM);

    REQUIRE(nullresult == std::nullopt);
}

TEST_CASE("SkipList:SimpleAllKeysInOrder:ExpectInOrderList",
          "[Required][SkipList]") {
    const unsigned int NUMBER_OF_ELEMENTS = 10;
    const unsigned int VALUE_OFFSET = 100;

    proj2::SkipList<unsigned, unsigned> skipList;
    std::vector<unsigned> expected;

    for (unsigned i = 0; i < NUMBER_OF_ELEMENTS; i++) {
        skipList.insert(i, (VALUE_OFFSET + i));
        expected.push_back(i);
    }

    REQUIRE(expected == skipList.allKeysInOrder());
}

TEST_CASE("SkipList:EraseOnEmptyThrows", "[Required][SkipList]") {
    proj2::SkipList<unsigned, unsigned> skipList;
    REQUIRE_THROWS(skipList.erase(0));
}

TEST_CASE("SkipList:EraseKey", "[Required][SkipList]") {
    proj2::SkipList<unsigned, unsigned> skipList;
    skipList.insert(0, 0);
    REQUIRE(skipList.size() == 1);
    REQUIRE(skipList.find(0).first == 0);
    skipList.erase(0);
    REQUIRE(skipList.empty());
    REQUIRE_THROWS(skipList.find(0));
}

}  // namespace
