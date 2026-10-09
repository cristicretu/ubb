package IsPrime;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;


public class AppTest_LongestSeq_IsPrimeStub {

    private LongSeq objLongestSequence;

    @Before
    public void setup()
    {
        List<Integer> l = Arrays.asList();
        objLongestSequence = new LongSeq(l);
    }


    @Test
    public void test_driver_for_SolveLongestSequence_with_IsPrimeStub() {
        System.out.println("Test driver for SolveLongestSequence");
        List<Integer> l = Arrays.asList(5, 7, 15, 2, 13, 23);
        objLongestSequence.setSequence(l);

        try {
            objLongestSequence.SolveLongestSequence();
            assertEquals(3, objLongestSequence.getStart());
            assertEquals(3, objLongestSequence.getLength());
        } catch (MyValueException e) {
            e.printStackTrace();
        }
    }


    @After
    public void tearDown(){
        objLongestSequence = null;
    }

}


