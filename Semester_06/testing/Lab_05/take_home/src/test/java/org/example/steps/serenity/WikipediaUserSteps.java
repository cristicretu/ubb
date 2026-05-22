package org.example.steps.serenity;

import org.example.pages.WikipediaArticlePage;
import org.example.pages.WikipediaHomePage;
import org.example.pages.WikipediaResultPage;
import net.thucydides.core.annotations.Step;
import net.thucydides.core.steps.ScenarioSteps;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.containsString;

public class WikipediaUserSteps extends ScenarioSteps {

    WikipediaHomePage homePage;
    WikipediaArticlePage articlePage;
    WikipediaResultPage resultPage;

    @Step
    public void is_on_the_home_page() {
        homePage.open();
    }

    @Step
    public void searches_for(String keyword) {
        homePage.enter_search_term(keyword);
    }

    @Step
    public void opens_article(String articleSlug) {
        articlePage.open_article(articleSlug);
    }

    @Step
    public void should_see_text(String expected) {
        assertThat(resultPage.getPageBodyText(), containsString(expected));
    }
}
