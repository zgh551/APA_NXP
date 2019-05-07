/*
 * link_list.h
 *
 *  Created on: 2019年4月25日
 *      Author: zhuguohua
 */

#ifndef UTILS_LINK_LIST_H_
#define UTILS_LINK_LIST_H_

#include "stdlib.h"
#include "derivative.h"
#include "node.h"

class LinkList {
public:
	LinkList();
	virtual ~LinkList();

	uint32_t Length();//返回链表个数
	void Add(ObstacleLocationPacket dat);
	void Delete(void);

	Node* getHeadNode();
	Property<LinkList,Node*,READ_ONLY> HeadNode;
private:
	uint32_t _list_length;
	Node* _head_node;//头节点
	Node* _end_node;//尾节点
	Node* _node;//
};

//class UltrasonicList:public LinkList<ObstacleLocationPacket>
//{
//
//};
#endif /* UTILS_LINK_LIST_H_ */