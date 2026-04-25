package IsPrime;

import java.util.Arrays;
import java.util.List;

public class LongSeq_VerifyIsPrimeMock {


    private List<Integer> l;
    private List<Integer> lSeq;
    private int start, length;

    private VerifyIsPrime verifyIsPrime;


    public LongSeq_VerifyIsPrimeMock(VerifyIsPrime verifyIsPrime){
        System.out.println("Longest sequence vida ...");
        this.verifyIsPrime = verifyIsPrime;
    }
    public void setSequence(List<Integer> l)
    {
        this.l=l;

    }
    public LongSeq_VerifyIsPrimeMock(List<Integer> newL, VerifyIsPrime verifyIsPrime)
    {
        this.lSeq = Arrays.asList();
        this.l = newL;
        this.start=-1;
        this.length=0;
        this.verifyIsPrime=verifyIsPrime;
        System.out.println("Longest sequence ...");
    }
    public int getStart()
    {
        return this.start;
    }
    public int getLength()
    {
        return this.length;

    }


    public void SolveLongestSequence() throws MyValueException
    {
        //System.out.println("in sir ... initial ... nr elemente=  "+this.l.size());
        int posI=-1, lengthI=0, i=0;
        int posF=-1, lengthF=0;
        //System.out.println("l.size="+this.l.size());
        while(i<this.l.size())
        {
            //if(isPrime(this.l.get(i).intValue())==true)
            //if(isPrimeStub(this.l.get(i).intValue())==true)
            if(verifyIsPrime.isPrime(this.l.get(i).intValue())==true)
            {
                if(posI==-1)
                {
                    posI=i;
                    lengthI=1;
                }
                else
                    lengthI++;
                //	System.out.println("inca prim...valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
            }
            else
            {
                if(lengthI>lengthF)
                {
                    lengthF=lengthI;
                    posF = posI;
                    //System.out.println("valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
                }
                posI=-1; lengthI=0;
            }
            //  System.out.println("valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
            i++;

        }
        if(lengthI>lengthF)
        {
            lengthF=lengthI;
            posF = posI;
        }
        //System.out.println("in metoda, la final, valorile sunt: pozF=" + Integer.toString(posF)+ " si lengthF="+Integer.toString(lengthF));

        this.start =posF;
        this.length=lengthF;
        //System.out.println("in sir ... la final ... nr elemente=  "+this.l.size());
    }



}
