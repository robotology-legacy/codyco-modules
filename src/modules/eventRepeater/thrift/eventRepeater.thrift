

service eventRepeater {
    /**
     * Raise an event on the output streaming port
     * @param event event to raise.
     *
     */
    bool sendEvent(1:string event);

    /**
     * Raise an event on the output streaming port
     * @param event event to raise.
     *
     * \note This is just an shorted alias for the sendEvent method
     */
    bool se(1:string event);
}
