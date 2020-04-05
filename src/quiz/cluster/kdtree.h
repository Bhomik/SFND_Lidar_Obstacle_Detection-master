/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
        : root(NULL)
    {}

    void insert_data(Node* node_ptr, uint depth, std::vector<float> point, int id)
    {

        if (node_ptr == NULL) // if
        {
            // Now node_ptr is pointing to node filled with point and id
            // and it has 2 child roots which are pointing to NULL.
            node_ptr = new Node(point, id);
        }
        else
        {
            uint current_dim = depth%2;

            if (point.at(current_dim) < node_ptr->point.at(current_dim))
            {
                insert_data(node_ptr->left, depth+1, point, id);
            }
            else
            {
                insert_data(node_ptr->right, depth+1, point, id);
            }

        }

    }

    void insert(std::vector<float> point, int id)
    {
        // root is a pointer to Node
        insert_data(root, 0, point, id);
    }

//    void insert(std::vector<float> point, int id)
//    {


//        std::cout << "point is : " << point[0] << " " << point[1] << std::endl;
//        std::cout << "Root is " << root << std::endl;

//        int depth = 0;
//        if (root == NULL)
//        {
//            root = new Node(point, id);
//        }
//        while(root!=NULL)
//        {
//            if(depth%2 == 0)          // depth = 0,2,4,6 comparisons to be done on x
//            {
//                if(point.at(0) < (root->point).at(0))
//                {
//                    // insert the node to left children
//                    //                    root = root->left;
//                    root->left = new Node(point,id);
//                    //                    depth++;

//                }
//                else
//                {
//                    // insert the node to right children
//                    //                    root = root->right;
//                    root->right = new Node(point, id);

//                }
//            }
//            else                    // depth = 1,3,5,7 comparisons to be done on y
//            {
//                if(point.at(1) < (root->point).at(1))
//                {
//                    // insert the node to left children
//                    //                    root = root->left;
//                    root->left = new Node(point,id);
//                }
//                else
//                {
//                    // insert the node to right children
//                    //                    root = root->right;
//                    root->right = new Node(point, id);
//                }
//            }
//        }
//        //        else if(depth%2 !=0)
//        std::cout << "root now is : " << (root->point).at(0) << "\t" << (root->point)[1] << std::endl;
//        std::cout << "root children are " << root->left << "\t" << root->right << std::endl;


//        //            if(id==0 || depth%2==0)
//        //            {

//        //            }
//        //            else
//        //            {
//        //            }

//        // TODO: Fill in this function to insert a new point into the tree
//        // the function should create a new node and place correctly with in the root

//    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        return ids;
    }


};




