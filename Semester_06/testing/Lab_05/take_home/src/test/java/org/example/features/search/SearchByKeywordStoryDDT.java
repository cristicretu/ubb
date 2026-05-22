package org.example.features.search;

import net.serenitybdd.junit.runners.SerenityParameterizedRunner;
import net.thucydides.core.annotations.Issue;
import net.thucydides.core.annotations.Managed;
import net.thucydides.core.annotations.Steps;
import net.thucydides.junit.annotations.Qualifier;
import net.thucydides.junit.annotations.UseTestDataFrom;
import org.example.steps.serenity.WikipediaUserSteps;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.openqa.selenium.WebDriver;

@RunWith(SerenityParameterizedRunner.class)
@UseTestDataFrom("src/test/resources/WikipediaSearchData.csv")
public class SearchByKeywordStoryDDT {

    @Managed(uniqueSession = true)
    public WebDriver webdriver;

    @Steps
    public WikipediaUserSteps user;

    private String caseName;
    private String keyword;
    private String expectedFragment;

    @Qualifier
    public String getQualifier() {
        return caseName;
    }

    @Issue("#WIKI-SEARCH")
    @Test
    public void searching_by_keyword_should_show_expected_text() {
        user.is_on_the_home_page();
        user.searches_for(keyword);
        user.should_see_text(expectedFragment);
    }

    public String getCaseName() { return caseName; }
    public void setCaseName(String caseName) { this.caseName = caseName; }

    public String getKeyword() { return keyword; }
    public void setKeyword(String keyword) { this.keyword = keyword; }

    public String getExpectedFragment() { return expectedFragment; }
    public void setExpectedFragment(String expectedFragment) { this.expectedFragment = expectedFragment; }
}
