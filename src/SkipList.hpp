#ifndef ___SKIP_LIST_HPP
#define ___SKIP_LIST_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace shindler::ics46::project2 {
    
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
    struct Node {
        K key{};
        bool keyValid{false};

        V value{};
        bool hasValue{false};

        Node* right{nullptr};
        Node* down{nullptr};

        Node() = default; // sentinel
        Node(const K& k, const V& v)
            : key(k), keyValid(true), value(v), hasValue(true) {}
        // Upper-level tower node (no value copy)
        Node(const K& k, Node* below)
            : key(k), keyValid(true), hasValue(false), down(below) {}
    };

    Node* topHead_;   // head sentinel of the topmost (always-empty) layer
    size_t height_;   // number of layers (>=2). Bottom layer is S_0.
    size_t size_;     // number of distinct keys

    [[nodiscard]] size_t layerLimit(size_t nextSize) const {
        if (nextSize <= 16) return 13;
        return 3 * static_cast<size_t>(std::ceil(std::log2(nextSize))) + 1;
    }

    void addEmptyTopLayer() {
        Node* newHead = new Node();
        newHead->down = topHead_;
        topHead_ = newHead;
        ++height_;
    }

    // Search path predecessors (top -> bottom). Also counts visited nodes.
    void collectPredecessors(const K& key,
                             std::vector<Node*>& preds,
                             size_t& visited) const {
        preds.clear();
        preds.reserve(height_);
        Node* cur = topHead_;
        while (cur) {
            ++visited; // being on this node at layer start
            while (cur->right && cur->right->keyValid && cur->right->key < key) {
                cur = cur->right;
                ++visited;
            }
            preds.push_back(cur);
            cur = cur->down;
        }
    }

    // Return bottom node for key or nullptr. Counts visited.
    Node* searchBottomNode(const K& key, size_t& visited) const {
        Node* cur = topHead_;
        while (cur) {
            ++visited;
            while (cur->right && cur->right->keyValid && cur->right->key < key) {
                cur = cur->right;
                ++visited;
            }
            if (cur->right && cur->right->keyValid && cur->right->key == key) {
                // descend to bottom along tower
                Node* hit = cur->right;
                while (hit->down) hit = hit->down;
                return hit;
            }
            cur = cur->down;
        }
        return nullptr;
    }

    void destroyAll() {
        // Delete layer by layer
        Node* layer = topHead_;
        while (layer) {
            Node* cur = layer;
            layer = layer->down;
            while (cur) {
                Node* next = cur->right;
                delete cur;
                cur = next;
            }
        }
        topHead_ = nullptr;
        height_ = 0;
        size_ = 0;
    }

   public:
    SkipList();
    SkipList(const SkipList&) = delete;
    SkipList(SkipList&&) = delete;
    SkipList& operator=(const SkipList&) = delete;
    SkipList& operator=(SkipList&&) = delete;
    ~SkipList();

    [[nodiscard]] size_t size() const noexcept;
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] size_t layers() const noexcept;
    [[nodiscard]] size_t height(const K& key) const;
    [[nodiscard]] const K& nextKey(const K& key) const;
    [[nodiscard]] const K& previousKey(const K& key) const;
    [[nodiscard]] std::pair<V&, size_t> find(const K& key);
    [[nodiscard]] std::pair<const V&, size_t> find(const K& key) const;
    std::optional<size_t> insert(const K& key, const V& value);
    [[nodiscard]] std::vector<K> allKeysInOrder() const;
    [[nodiscard]] bool isSmallestKey(const K& key) const;
    [[nodiscard]] bool isLargestKey(const K& key) const;
    void erase(const K& key);
};


template <typename K, typename V>
SkipList<K, V>::SkipList()
    : topHead_(new Node()), height_(2), size_(0) {
    Node* bottomHead = new Node();
    topHead_->down = bottomHead;
}

template <typename K, typename V>
SkipList<K, V>::~SkipList() {
    destroyAll();
}

template <typename K, typename V>
size_t SkipList<K, V>::size() const noexcept {
    return size_;
}

template <typename K, typename V>
bool SkipList<K, V>::empty() const noexcept {
    return size_ == 0;
}

template <typename K, typename V>
size_t SkipList<K, V>::layers() const noexcept {
    return height_;
}

template <typename K, typename V>
size_t SkipList<K, V>::height(const K& key) const {
    size_t h = 0;
    Node* cur = topHead_;
    while (cur) {
        while (cur->right && cur->right->keyValid && cur->right->key < key)
            cur = cur->right;
        if (cur->right && cur->right->keyValid && cur->right->key == key) {
            ++h;
            cur = cur->right->down;
        } else {
            cur = cur->down;
        }
    }
    if (h == 0) throw std::out_of_range("Key not found");
    return h;
}

template <typename K, typename V>
const K& SkipList<K, V>::nextKey(const K& key) const {
    size_t dummy = 0;
    Node* bottomNode = searchBottomNode(key, dummy);
    if (!bottomNode) throw std::out_of_range("Key not found");


    Node* bottomHead = topHead_;
    while (bottomHead->down) bottomHead = bottomHead->down;
    Node* cur = bottomHead;
    while (cur->right && cur->right->keyValid && cur->right->key <= key)
        cur = cur->right;
    if (!cur->right || !cur->right->keyValid)
        throw std::out_of_range("No next key");
    return cur->right->key;
}

template <typename K, typename V>
const K& SkipList<K, V>::previousKey(const K& key) const {
    size_t dummy = 0;
    Node* bottomNode = searchBottomNode(key, dummy);
    if (!bottomNode) throw std::out_of_range("Key not found");
    Node* bottomHead = topHead_;
    while (bottomHead->down) bottomHead = bottomHead->down;

    Node* cur = bottomHead->right;
    Node* prev = nullptr;
    while (cur && cur->keyValid && cur->key < key) {
        prev = cur;
        cur = cur->right;
    }
    if (!cur || cur->key != key) throw std::logic_error("Structure error");
    if (!prev) throw std::out_of_range("No previous key");
    return prev->key;
}

template <typename K, typename V>
std::pair<const V&, size_t> SkipList<K, V>::find(const K& key) const {
    size_t visited = 0;
    Node* bottomNode = searchBottomNode(key, visited);
    if (!bottomNode) throw std::out_of_range("Key not found");
    return {bottomNode->value, visited};
}

template <typename K, typename V>
std::pair<V&, size_t> SkipList<K, V>::find(const K& key) {
    size_t visited = 0;
    Node* bottomNode = searchBottomNode(key, visited);
    if (!bottomNode) throw std::out_of_range("Key not found");
    return {bottomNode->value, visited};
}

template <typename K, typename V>
std::optional<size_t> SkipList<K, V>::insert(const K& key, const V& value) {
    size_t visited = 0;
    std::vector<Node*> preds;
    collectPredecessors(key, preds, visited); // top -> bottom
    Node* bottomPred = preds.back();
    Node* candidate = bottomPred->right;
    if (candidate && candidate->keyValid && candidate->key == key) {
        return std::nullopt; // already exists
    }

    size_t nextSize = size_ + 1;
    size_t limit = layerLimit(nextSize);
    size_t towerLevels = 1;
    while (towerLevels < limit && flipCoin(key, towerLevels - 1))
        ++towerLevels;

    while (towerLevels >= height_) { 
        addEmptyTopLayer();
        preds.insert(preds.begin(), topHead_);
    }

    Node* bottomNode = new Node(key, value);
    bottomNode->right = bottomPred->right;
    bottomPred->right = bottomNode;

    Node* below = bottomNode;
    size_t built = 1;
    for (int i = static_cast<int>(preds.size()) - 2; // one above bottom
         i >= 0 && built < towerLevels;
         --i, ++built) {
        Node* pred = preds[static_cast<size_t>(i)];
        Node* upper = new Node(key, below); 
        upper->right = pred->right;
        pred->right = upper;
        below = upper;
    }

    ++size_;
    return visited;
}

template <typename K, typename V>
std::vector<K> SkipList<K, V>::allKeysInOrder() const {
    std::vector<K> out;
    Node* layer = topHead_;
    while (layer->down) layer = layer->down; 
    Node* cur = layer->right;
    while (cur) {
        if (cur->keyValid) out.push_back(cur->key);
        cur = cur->right;
    }
    return out;
}

template <typename K, typename V>
bool SkipList<K, V>::isSmallestKey(const K& key) const {
    size_t dummy = 0;
    Node* bottomNode = searchBottomNode(key, dummy);
    if (!bottomNode) throw std::out_of_range("Key not found");
    Node* bottomHead = topHead_;
    while (bottomHead->down) bottomHead = bottomHead->down;
    Node* first = bottomHead->right;
    if (!first || !first->keyValid) throw std::logic_error("Empty structure mismatch");
    return first->key == key;
}

template <typename K, typename V>
bool SkipList<K, V>::isLargestKey(const K& key) const {
    size_t dummy = 0;
    Node* bottomNode = searchBottomNode(key, dummy);
    if (!bottomNode) throw std::out_of_range("Key not found");
    Node* bottomHead = topHead_;
    while (bottomHead->down) bottomHead = bottomHead->down;
    Node* cur = bottomHead->right;
    Node* last = nullptr;
    while (cur) {
        if (cur->keyValid) last = cur;
        cur = cur->right;
    }
    if (!last) throw std::logic_error("Empty structure mismatch");
    return last->key == key;
}

template <typename K, typename V>
void SkipList<K, V>::erase(const K& key) {
    bool removed = false;
    Node* cur = topHead_;
    while (cur) {
        while (cur->right && cur->right->keyValid && cur->right->key < key)
            cur = cur->right;
        if (cur->right && cur->right->keyValid && cur->right->key == key) {
            Node* doomed = cur->right;
            cur->right = doomed->right;
            delete doomed;
            removed = true;
        }
        cur = cur->down;
    }
    if (!removed) throw std::out_of_range("Key not found");
    --size_;
}

}  // namespace shindler::ics46::project2
#endif
