// TreeNode.h
#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <vector>
#include <memory>

class TreeNode
{
public:
    TreeNode(const std::vector<float> &val, std::shared_ptr<TreeNode> parent);

    void addChild(std::shared_ptr<TreeNode> child);
    std::vector<float> val;
    std::shared_ptr<TreeNode> parent;
    std::vector<std::shared_ptr<TreeNode>> children;
};

#endif // TREE_NODE_H