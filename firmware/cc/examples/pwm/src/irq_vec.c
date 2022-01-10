void handle_tim1();

void (*const isr_vector[3])(void) = {
    &handle_tim1,       /* Timer 1 interrupt */
    0x0                 /* Timer 2 interrupt */
};

