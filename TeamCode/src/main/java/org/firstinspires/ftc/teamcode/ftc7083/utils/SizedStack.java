package org.firstinspires.ftc.teamcode.ftc7083.utils;

import java.util.Stack;

/**
 * A stack that has a maximum size. If the maximum size is exceeded, then the oldest element on
 * the stack is removed.
 *
 * @param <T> the type of element to include in the stack.
 */
public class SizedStack<T> extends Stack<T> {
    private final int maxSize;

    /**
     * Instantiates a new sized stack.
     *
     * @param size the maximum number of elements to include on the stack.
     */
    public SizedStack(int size) {
        super();
        this.maxSize = size;
    }

    @Override
    public T push(T object) {
        //If the stack is too big, remove elements until it's the right size.
        while (this.size() >= maxSize) {
            this.remove(0);
        }
        return super.push(object);
    }
}
