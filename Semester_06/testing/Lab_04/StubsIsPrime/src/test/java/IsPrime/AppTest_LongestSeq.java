package test;

import java.util.Arrays;
import java.util.List;



import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import IsPrime.LongSeq;
import IsPrime.MyValueException;

public class AppTest_LongestSeq {
	
private LongSeq objLongestSequence;
	
	@Before
	public void setup()
	{
		List<Integer> l = Arrays.asList();
		objLongestSequence = new LongSeq(l);
	}


	@Test
	public void test_TC_1_EC() {
		System.out.println("Test number 1");
		List<Integer> l = Arrays.asList(6, 4, 12, 14, 8);
		objLongestSequence.setSequence(l);
		try {			
			objLongestSequence.SolveLongestSequence();			
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()) );
			//assertEquals(-1, objLongestSequence.getStart());
			//assertEquals(0, objLongestSequence.getLength());
			
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@Test
	public void test_TC_2_EC() {
		System.out.println("Test number 2");
		List<Integer> l = Arrays.asList();
		objLongestSequence.setSequence(l);
		try {			
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(-1, objLongestSequence.getStart());
			assertEquals(0, objLongestSequence.getLength());
			
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@Test
	public void test_TC_3_EC() {
		System.out.println("Test number 3");
		List<Integer> l = Arrays.asList(2, 3, 0, 6, 4);
		objLongestSequence.setSequence(l);
		try {
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(0, objLongestSequence.getStart());
			assertEquals(2,objLongestSequence.getLength());
			
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@Test
	public void test_TC_4_EC() {
		System.out.println("Test number 4");
		List<Integer> l = Arrays.asList(6, 4, 0, 2, 3);
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(3, objLongestSequence.getStart());
			assertEquals(2, objLongestSequence.getLength());
			
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@Test
	public void test_TC_5_EC(){
		System.out.println("Test number 5");
		List<Integer> l = Arrays.asList(6, 2, 3, 0, 8);
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(1, objLongestSequence.getStart());
			assertEquals(2, objLongestSequence.getLength());
			
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}	
	@Test
	public void test_TC_6_EC(){
		System.out.println("Test number 6");
		List<Integer> l = Arrays.asList(2, 3, 5, 4, 2, 3, 5, 4, 8, 10);
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(0, objLongestSequence.getStart());
			assertEquals(3, objLongestSequence.getLength());
			
			//System.out.println("Terminat test number 6");
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	
	
	@Test
	public void test_TC_1_BVA(){
		System.out.println("Test number 6");
		List<Integer> l = Arrays.asList();
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(-1, objLongestSequence.getStart());
			assertEquals(0, objLongestSequence.getLength());
			
			//System.out.println("Terminat test number 6");
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	//Test
	public void test_TC_2_BVA(){
		System.out.println("Test number 6");
		List<Integer> l = Arrays.asList();
		int i=-1, j=1;
		for (i=0;i<Integer.MAX_VALUE-1;i++)
			l.add(Integer.valueOf(j));
			
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(0, objLongestSequence.getStart());
			assertEquals(Integer.MAX_VALUE, objLongestSequence.getLength());
			
			//System.out.println("Terminat test number 6");
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@Test
	public void test_TC_3_BVA(){
		System.out.println("Test number 6");
		List<Integer> l = Arrays.asList(7);	
			
		objLongestSequence.setSequence(l);
		try {						
			objLongestSequence.SolveLongestSequence();
			//System.out.println("pozF=" + Integer.toString(objLongestSequence.getStart())+ " si lengthF="+Integer.toString(objLongestSequence.getLength()));
			assertEquals(0, objLongestSequence.getStart());
			assertEquals(1, objLongestSequence.getLength());
			
			//System.out.println("Terminat test number 6");
		} catch (MyValueException e) {
			e.printStackTrace();
		}		
	}
	@After
	public void tearDown(){
		objLongestSequence = null;
	}

}
