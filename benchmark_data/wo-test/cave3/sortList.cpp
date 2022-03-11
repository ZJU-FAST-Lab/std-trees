#include <iostream>
using std::cin;
using std::cout;

struct Node 
{
  int value;
  Node* child;
  Node() : value(-1), child(nullptr) {}
  Node(int n) : value(n), child(nullptr) {}
};

Node* merge(Node* h1, Node* h2)
{
  Node* pre_head = new Node();
  Node* head = pre_head;
  while (h1 != nullptr && h2 != nullptr) 
  {
    if (h1->value > h2->value) // large values first
    {
      head->child = h1;
      h1 = h1->child;
    }
    else 
    {
      head->child = h2;
      h2 = h2->child;
    }
    head = head->child;
  }
  if (h1 == nullptr) head->child = h2;
  else head->child = h1;

  return pre_head->child;
}

Node* sort(Node* head)
{
  if (head == nullptr || head->child == nullptr) return head;
  Node* slow(head);
  Node* fast(head);
  while (fast->child != nullptr && fast->child->child != nullptr) 
  {
    slow = slow->child;
    fast = fast->child->child;
  }
  Node* middle(slow->child);
  slow->child = nullptr;
  Node* h1 = sort(head);
  Node* h2 = sort(middle);
  return merge(h1, h2);
}

int main () 
{
  int k(-1), val(-1);
  cin >> k; 
  Node* pre_head = new Node();
  Node* p = pre_head;
  for (int i=0; i<k; ++i)
  {
    cin >> val;
    Node* node = new Node(val);
    p->child = node;
    p = p->child;
  }
  cout << "before sorting: \n";
  p = pre_head;
  while (p->child)
  {
    p = p->child;
    cout << p->value << " ";
  }
  cout << "\n";

  Node* res = sort(pre_head->child);

  cout << "after sorting: \n";
  while (res)
  {
    cout << res->value << " ";
    Node* d(res);
    delete d;
    res = res->child;
  }
  cout << "\n";

  return 0;
}
