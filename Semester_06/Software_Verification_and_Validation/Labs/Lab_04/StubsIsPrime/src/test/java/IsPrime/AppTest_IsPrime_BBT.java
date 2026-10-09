package IsPrime;



//import static org.junit.Assert.assertEquals;
//import static org.junit.Assert.assertTrue;

//import org.junit.After;
//import org.junit.Before;
//import org.junit.Ignore;
//import org.junit.Test;


import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

/**
 * Unit test for simple App.
 */
public class AppTest_IsPrime_BBT
{

    private VerifyIsPrime objVerifyIsPrime;

    @Before
    public void setup()
    {
        objVerifyIsPrime = new VerifyIsPrime();
    }

    @Test
    public void test_TC_1_EC() {
        try
        {
            assert( true == objVerifyIsPrime.isPrime(5));
        }
        catch (MyValueException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void test_TC_2_EC(){
        try
        {
            assert (false== objVerifyIsPrime.isPrime(14));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }
    }
    @Test
    public void test_TC_3_EC()
    {
        try
        {
            assert (false ==objVerifyIsPrime.isPrime(-5));
        }
        catch(MyValueException e)
        {
            assert ("data not valid".equals(e.getMessage()));
        }
    }

    @Test(expected = IsPrime.MyValueException.class)
    public void test_TC_3_EC_or() throws MyValueException
    {
        objVerifyIsPrime.isPrime(-5);
    }

    @Test
    public void test_TC_1_BVA()
    {
        try
        {
            assert (false == objVerifyIsPrime.isPrime(0));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }
    }
    @Test
    public void test_TC_2_BVA()
    {
        try
        {								//MAX_VALUE = 2147483647 -prime
            assert (true== objVerifyIsPrime.isPrime(Integer.MAX_VALUE));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }

    }
    @Test
    public void test_TC_3_BVA()
    {
        try
        {
            assert (false==objVerifyIsPrime.isPrime(-1));
        }
        catch(MyValueException e)
        {
            //e.printStackTrace();
            assert ("data not valid".equals(e.getMessage()));
        }
    }
    @Test
    public void test_TC_4_BVA()
    {
        try
        {
            assert (false== objVerifyIsPrime.isPrime(1));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }
    }

    @Test
    public void test_TC_5_BVA()
    {
        try
        {								//MAX_VALUE = 2147483647-1=2147483646  -prime
            assert (false== objVerifyIsPrime.isPrime(Integer.MAX_VALUE-1));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }

    }
    @Ignore
    //@Test
    public void test_TC_6_BVA()
    {
        try
        {								//MAX_VALUE = 2147483647-1=2147483646  -prime
            assert (false==objVerifyIsPrime.isPrime(Integer.MAX_VALUE+1));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }

    }
    @Test
    public void test_TC_7_special()
    {
        try
        {
            assert (true== objVerifyIsPrime.isPrime(5));
        }
        catch(MyValueException e)
        {
            e.printStackTrace();
        }
    }

    @After
    public void tearDown(){
        objVerifyIsPrime = null;
    }







}
