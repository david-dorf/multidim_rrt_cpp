#include <vector>

class TreeNode
{
public:
    std::vector<float> val;
    TreeNode *parent;
    std::vector<TreeNode *> children;
    TreeNode(std::vector<float> val, TreeNode *parent) : val(val), parent(parent) {}
    void add_child(TreeNode *child)
    {
        children.push_back(child);
    }
};
