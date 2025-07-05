#ifndef ___SKIP_LIST_HPP
#define ___SKIP_LIST_HPP

#include <cmath>  // for log2
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace shindler::ics46::project2 {

/**
 * flipCoin -- NOTE: Only read if you are interested in how the
 * coin flipping works.
 *
 * @brief Uses the bitwise representation of the key to simulate
 * the flipping of a deterministic coin.
 *
 * This function looks at the bitwise representation to determine
 * how many layers it occupies in the skip list. It takes the bitwise
 * XOR of each byte in a 32-bit unsigned number and uses the index
 * `previousFlips` to determine the truth value. It's best to look at an example
 * and to start with values that fit into one byte.
 *
 * Let's start with the function call `flipCoin(0, 0)`. This call is
 * asking whether or not we should insert the key `0` into layer `1`
 * (because `previousFlips` represents the number of previous flips).
 *
 * This function will get the bitwise representation of 0:
 *
 *        00000000 00000000 00000000 00000000
 *
 * All bytes are XOR'd together:
 *
 *
 *  c = 0 ^ 0 ^ 0 ^ 0
 *
 * Then the index `previousFlips` is used to obtain the bit in the
 * `previousFlips % 8` position.
 *
 * So the return value is `0 & 1 = 0`, since the value in the zeroth position
 * (obtained by bitmasking the value (1 << 0) at the bottom of the function
 * is 0.
 * Thus, this value `0` should never get added beyond the bottom-most layer.
 *
 * Before:
 *
 * S_1: -inf ----> inf
 * S_0: -inf ----> inf
 *
 * And after 0 is inserted
 *
 * S_1: -inf --------> inf
 * S_0: -inf --> 0 --> inf
 *
 * Let's look at something more interesting, like the call `flipCoin(5, 0)`.
 * Remember the binary representation for 5 is 00000101.
 *
 * c = 0 ^ 0 ^ 0 ^ 00000101 = 00000101
 *
 * Now we get the bit at 0th position (from `previousFlips`).
 *
 * 00000101 & 1 = 1. Thus, `5` DOES get inserted into the next layer,
 * layer 1.
 *
 * So the skip list before this insertion might look like:
 *
 * S_1: -inf ----> inf
 * S_0: -inf ----> inf
 *
 * And after it would look like:
 *
 * S_2: -inf --------> inf
 * S_1: -inf --> 5 --> inf
 * S_0: -inf --> 5 --> inf
 *
 * Observe that a new layer had to get created, because there should always
 * be an empty layer at the top.
 *
 * Since 5 got inserted into the next layer, we need to flip again to see if
 * it should get propagated once more.
 *
 * `flipCoin(5, 1)` this will produce a result of `0`, since 00000101 & (1 << 1)
 * = 0.
 *
 * Thus, `5` will not be pushed up to the next layer.
 *
 * @param key The integer key which will be added to the skip list
 * @param previousFlips The number of previous flips for this key
 * @return true simulates a "heads" from a coin flip
 * @return false simulates a "tails" from a coin flip
 */
enum class IntegerByteOffsets {
    Byte0 = 24,
    Byte1 = 16,
    Byte2 = 8,
    Byte3 = 0,
};

constexpr inline std::byte flipCoinByteSelector(uint32_t key,
                                                IntegerByteOffsets byte) {
    const uint32_t BYTE_SELECTOR{0xFF};
    auto byteAsInteger{static_cast<uint32_t>(byte)};
    return static_cast<std::byte>((key & (BYTE_SELECTOR << byteAsInteger)) >>
                                  byteAsInteger);
}

const uint32_t NUMBER_OF_BITS_IN_BYTE = 8;

constexpr inline bool flipCoin(unsigned int key, size_t previousFlips) {
    std::byte firstByte{flipCoinByteSelector(key, IntegerByteOffsets::Byte0)};
    std::byte secondByte{flipCoinByteSelector(key, IntegerByteOffsets::Byte1)};
    std::byte thirdByte{flipCoinByteSelector(key, IntegerByteOffsets::Byte2)};
    std::byte fourthByte{flipCoinByteSelector(key, IntegerByteOffsets::Byte3)};

    std::byte hash{firstByte ^ secondByte ^ thirdByte ^ fourthByte};

    auto bitToSelect = std::byte{
        static_cast<uint8_t>(1 << (previousFlips % NUMBER_OF_BITS_IN_BYTE))};

    return std::to_integer<uint8_t>(hash & bitToSelect) != 0;
}

/**
 * @brief Works the same as the integer version above, except
 * it XORs chars in a string instead of the first four
 * bytes of an integer.
 *
 * @param key key that will be inserted into the skip list
 * @param previousFlips number of previous flips for this key
 * @return true simulates a "heads" from a coin flip
 * @return false simulates a "tails" from a coin flip
 */
constexpr inline bool flipCoin(const std::string& key, size_t previousFlips) {
    std::byte hash{};

    for (auto character : key) {
        hash ^= static_cast<std::byte>(character);
    }

    std::byte bitToSelect{
        static_cast<uint8_t>(1 << (previousFlips % NUMBER_OF_BITS_IN_BYTE))};

    return std::to_integer<uint8_t>(hash & bitToSelect) != 0;
}

template <typename K, typename V>
class SkipList {
   private:
    // private variables go here.

   public:
    SkipList();

    // You DO NOT need to implement a copy constructor or an assignment
    // operator. These are only here to make sure that the compiler stops
    // any unsopported behavior. For those of you that took 45c last quarter
    // you'll probably remember the incident where the copy was not implement
    // and it was doing a shallow copy which caused a bunch of issues. This
    // will prevent that (in theory).
    SkipList(const SkipList&) = delete;
    SkipList(SkipList&&) = delete;
    SkipList& operator=(const SkipList&) = delete;
    SkipList& operator=(SkipList&&) = delete;

    ~SkipList();

    // How many distinct keys are in the skip list?
    [[nodiscard]] size_t size() const noexcept;

    // Does the Skip List contain zero keys?
    [[nodiscard]] bool empty() const noexcept;

    // How many layers are in the skip list?
    // Note that an empty Skip List has two layers by default,
    // the "base" layer S_0 and the top layer S_1.
    //
    // [S_1: Top layer]    -inf ------> inf
    // [S_0: Bottom layer] -inf ------> inf
    //
    // This "empty" Skip List has two layers and a height of one.
    [[nodiscard]] size_t layers() const noexcept;

    // What is the height of this key, assuming the "base" layer S_0
    // contains keys with a height of 1?
    // For example, "0" has a height of 1 in the following skip list.
    //
    // [S_1]  -inf --------> inf
    // [S_0]  -inf --> 0 --> inf
    //
    // Throw an exception if this key is not in the Skip List.
    [[nodiscard]] size_t height(const K& key) const;

    // If this key is in the SkipList and there is a next largest key
    // return the next largest key.
    // This function should throw a std::out_of_range if either the key doesn't
    // exist or there is no subsequent key. A consequence of this is that this
    // function will throw a std::out_of_range if *k* is the *largest* key in
    // the Skip List.
    [[nodiscard]] const K& nextKey(const K& key) const;
    [[nodiscard]] const K& previousKey(const K& key) const;

    // These return the value associated with the given key and the number
    // of elements accessed while finding the element.<D-s>
    //
    // Throw a std::out_of_range if the key does not exist.
    [[nodiscard]] std::pair<V&, size_t> find(const K& key);
    [[nodiscard]] std::pair<const V&, size_t> find(const K& key) const;

    // Return the number of accesses made during insert or std::nullopt if
    // it failed
    // If the key already exists, do not insert one -- return std::nullopt;
    //
    // See the project write-up for conditions under which the key
    // should be "bubbled up" to the next layer.
    std::optional<size_t> insert(const K& key, const V& value);

    // Return a vector containing all inserted keys in increasing order.
    [[nodiscard]] std::vector<K> allKeysInOrder() const;

    // Is this the smallest key in the SkipList? Throw a std::out_of_range
    // if the key *k* does not exist in the Skip List.
    [[nodiscard]] bool isSmallestKey(const K& key) const;

    // Is this the largest key in the SkipList? Throw a std::out_of_range
    // if the key *k* does not exist in the Skip List.
    [[nodiscard]] bool isLargestKey(const K& key) const;

    // Erase the given key from the skip list. Throw a std::out_of_range
    // if the key *key* does not exist in the SkipList
    void erase(const K& key);
};

template <typename K, typename V>
SkipList<K, V>::SkipList() {
    // TODO - your implementation goes here!
}

template <typename K, typename V>
SkipList<K, V>::~SkipList() {
    // TODO - your implementation goes here!
}

template <typename K, typename V>
size_t SkipList<K, V>::size() const noexcept {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
bool SkipList<K, V>::empty() const noexcept {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
size_t SkipList<K, V>::layers() const noexcept {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
size_t SkipList<K, V>::height(const K& key) const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
const K& SkipList<K, V>::nextKey(const K& key) const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
const K& SkipList<K, V>::previousKey(const K& key) const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
std::pair<const V&, size_t> SkipList<K, V>::find(const K& key) const {
    // TODO - your implementation goes here!
    return {{}, {}};
}

template <typename K, typename V>
std::pair<V&, size_t> SkipList<K, V>::find(const K& key) {
    // TODO - your implementation goes here!
    auto* tmp_value = new V;  // Stub to make the project compile by default!
    return {*tmp_value, {}};
}

template <typename K, typename V>
std::optional<size_t> SkipList<K, V>::insert(const K& key, const V& value) {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
std::vector<K> SkipList<K, V>::allKeysInOrder() const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
bool SkipList<K, V>::isSmallestKey(const K& key) const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
bool SkipList<K, V>::isLargestKey(const K& key) const {
    // TODO - your implementation goes here!
    return {};
}

template <typename K, typename V>
void SkipList<K, V>::erase(const K& key) {
    // TODO -
}

}  // namespace shindler::ics46::project2
#endif
