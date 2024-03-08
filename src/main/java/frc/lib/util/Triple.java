package frc.lib.util;

    public class Triple<F, M, L> {
        private final F f;
        private final M m;
        private final L l;
        
        public Triple(F f, M m, L l){
            this.f = f;
            this.m = m;
            this.l = l;
        }

        public F getFirst(){return f;}
        public M getMiddle(){return m;}
        public L getLast(){return l;}

        public static <F, M, L> Triple<F, M, L> of(F f, M m, L l) {
            return new Triple<F, M, L>(f,m, l);
        }
    }