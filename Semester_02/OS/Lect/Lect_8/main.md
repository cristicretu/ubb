pthread_mutex_t
- init
- lock
- trylock
- unlock
- destroy


readers     | writers
rdlock(&l)  | wrlock(&l)
// consult  | // change
// critical | // critical
// vars     | // vars
unlock(&l)  | unlock(&l)


###  waiting

lock(&m)
while (ceva) {
    wait(&c, &m); -> unlock(&m), wait to temp, lock(&m)
    // do something
}
unlock(&m);

### notifier

lock(&m)
if (ceva) {
    signal(&c);
}
// do something
unlock(&l);

## sem_t

init
wait
post
destroy
