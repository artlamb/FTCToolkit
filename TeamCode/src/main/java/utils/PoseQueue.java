package utils;

import common.Logger;

public class PoseQueue {
    private final Pose[] queue;
    private final int capacity;
    private int front;
    private int rear;
    private int size;

    // Constructor to initialize the queue
    public PoseQueue(int capacity) {
        this.capacity = capacity;
        this.queue = new Pose[capacity];
        this.front = 0;
        this.rear = -1;
        this.size = 0;
    }

    // Enqueue operation
    public void enqueue(Pose pose) {

        // disguard redundant poses.
        if (size > 0 &&
                Math.floor(pose.getX()*100) == Math.floor(queue[front].getX()*100) &&
                Math.floor(pose.getY()*100) == Math.floor(queue[front].getY()*100)) {
            Logger.warning("redundant");
        }

        rear = (rear + 1) % capacity;
        queue[rear] = pose;
        if (isFull()) {
            front = (front + 1) % capacity;
        } else {
            size++;
        }
    }

    // Peek element
    public Pose peek(int index) {
        if (isEmpty()) {
            Logger.warning("Queue is empty. Nothing to peek.");
            return null;
        }

        int i = (front + index) % capacity;
        return queue[i];
    }

    // Check if the queue is full
    public boolean isFull() {
        return size == capacity;
    }

    // Check if the queue is empty
    public boolean isEmpty() {
        return size == 0;
    }

    public int size() {
        return size;
    }

    // Display the queue elements
    public void display() {
        Logger.message("Queue:");
        for (int i = 0; i < size(); i++) {
            Logger.message(peek(i).toString());
        }
    }

    // Main method for testing
    public void test() {
        PoseQueue cq = this;

        cq.enqueue(new Pose(10, 0, 0));
        cq.enqueue(new Pose(20, 0, 0));
        cq.enqueue(new Pose(30, 0, 0));
        cq.enqueue(new Pose(40, 0, 0));
        cq.enqueue(new Pose(50, 0, 0));

        cq.display();

        cq.enqueue(new Pose(60, 0, 0)); // Should say queue is full
        cq.enqueue(new Pose(70, 0, 0));
        cq.enqueue(new Pose(80, 0, 0));
        cq.enqueue(new Pose(90, 0, 0));
        cq.enqueue(new Pose(100, 0, 0));
        cq.enqueue(new Pose(110, 0, 0));
        cq.display();

        Logger.message("Peek element: " + cq.peek(1));
    }
}