package IsPrime;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class AppTest_LongestSeq_IsPrimeMock {

    private LongSeq_VerifyIsPrimeMock objLongestSequence;

    @Mock
    private VerifyIsPrime verifyIsPrime;

    @Before
    public void setup()
    {
        List<Integer> l = Arrays.asList();
        verifyIsPrime = mock(VerifyIsPrime.class);
        objLongestSequence = new LongSeq_VerifyIsPrimeMock(verifyIsPrime);
    }

    @Test
    public void test_driver_for_SolveLongestSequence_with_Mock() {
        System.out.println("Test driver for SolveLongestSequence");
        try {
            when(verifyIsPrime.isPrime(5)).thenReturn(true);
            when(verifyIsPrime.isPrime(7)).thenReturn(true);
            when(verifyIsPrime.isPrime(15)).thenReturn(false);
            when(verifyIsPrime.isPrime(2)).thenReturn(true);
            when(verifyIsPrime.isPrime(13)).thenReturn(true);
            when(verifyIsPrime.isPrime(23)).thenReturn(true);
        }catch (MyValueException e) {
                e.printStackTrace();
        }
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

}
