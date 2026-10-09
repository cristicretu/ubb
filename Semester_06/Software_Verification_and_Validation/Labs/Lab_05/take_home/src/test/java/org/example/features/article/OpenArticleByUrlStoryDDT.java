package org.example.features.article;

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
@UseTestDataFrom("src/test/resources/WikipediaArticleData.csv")
public class OpenArticleByUrlStoryDDT {

    @Managed(uniqueSession = true)
    public WebDriver webdriver;

    @Steps
    public WikipediaUserSteps user;

    private String caseName;
    private String articleSlug;
    private String expectedFragment;

    @Qualifier
    public String getQualifier() {
        return caseName;
    }

    @Issue("#WIKI-ARTICLE")
    @Test
    public void opening_article_by_url_should_show_expected_text() {
        user.opens_article(articleSlug);
        user.should_see_text(expectedFragment);
    }

    public String getCaseName() { return caseName; }
    public void setCaseName(String caseName) { this.caseName = caseName; }

    public String getArticleSlug() { return articleSlug; }
    public void setArticleSlug(String articleSlug) { this.articleSlug = articleSlug; }

    public String getExpectedFragment() { return expectedFragment; }
    public void setExpectedFragment(String expectedFragment) { this.expectedFragment = expectedFragment; }
}
